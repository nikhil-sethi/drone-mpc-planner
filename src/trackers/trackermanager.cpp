#include "trackermanager.h"
using namespace cv;
using namespace std;

namespace tracking {

void TrackerManager::init(std::ofstream *logger,string replay_dir_,VisionData *visdat, CameraView *camview) {
    _visdat = visdat;
    _logger = logger;
    _camview = camview;
    replay_dir = replay_dir_;

    if (pparams.has_screen || pparams.video_result || pparams.video_cuts) {
        enable_viz_max_points = false;
        enable_viz_diff = true;
    }

    deserialize_settings();
    read_false_positives();

    _dtrkr = new DroneTracker();
    _dtrkr->init(_logger,_visdat,motion_thresh,1);
    _trackers.push_back(_dtrkr);

    (*_logger) << "trkrs_state;n_trackers;n_blobs;";
#ifdef PROLILING
    (*_logger) << "dur1;dur2;dur3;dur4;dur5;";
#endif
    initialized = true;
}

void TrackerManager::update(double time, bool drone_is_active) {

#ifdef PROLILING
    auto profile_t0 = std::chrono::high_resolution_clock::now();
    auto dur1=0,dur2=0,dur3=0,dur4=0;
#endif
    prep_vizs();

#ifdef PROLILING
    dur1 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - profile_t0).count();
#endif
    if (_mode != mode_idle) {
        update_max_change_points();
#ifdef PROLILING
        dur2 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - profile_t0).count();
#endif
        collect_static_ignores();
        erase_dissipated_fps(time);

#ifdef PROLILING
        dur3 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - profile_t0).count();
#endif
        match_blobs_to_trackers(drone_is_active && !_dtrkr->inactive(),time);
#ifdef PROLILING
        dur4 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - profile_t0).count();
#endif
    }

    update_trackers(time,_visdat->frame_id, drone_is_active);
#ifdef PROLILING
    auto dur5 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - profile_t0).count();
#endif

    (*_logger) << static_cast<int16_t>(_mode) << ";" << _trackers.size() << ";" << _blobs.size() << ";";
#ifdef PROLILING
    (*_logger) << dur1 << ";"
               << dur2 << ";"
               << dur3 << ";"
               << dur4 << ";"
               << dur5 << ";";
#endif

    finish_vizs();
}

//Pats Blob finder. Looks for a limited number of
//maxima in the motion image that are higher than a threshold, the area around the maximum
//is segmented from the background motion noise, and seen as a blob. In special cases it then
//tries if this blob can be further splitted if necessary.
void TrackerManager::update_max_change_points() {
    cv::Mat diff = _visdat->diffL_small.clone();
    _blobs.clear();

    vizs_maxs.clear();

    bool enable_insect_drone_split = false;
    float drn_ins_split_thresh = 0;

    if (_mode == mode_locate_drone || !_dtrkr->image_predict_item().valid)
        roi_radius = 15;
    else
        roi_radius = ceilf(_dtrkr->image_predict_item().size * 0.95f) / pparams.imscalef;
    roi_radius = std::clamp(roi_radius,10,100);

    if (target_insecttracker()) {
        auto image_predict_item = target_insecttracker()->image_predict_item();
        if (_dtrkr->image_predict_item().valid &&
                image_predict_item.valid &&
                norm(_dtrkr->image_predict_item().pt() - image_predict_item.pt()) < roi_radius) {
            enable_insect_drone_split = true;
            drn_ins_split_thresh = image_predict_item.pixel_max*0.2f;
        }
    }

    Mat motion_noise_mapL = _visdat->motion_noise_mapL_small;
    if (!motion_noise_mapL.cols)
        motion_noise_mapL = cv::Mat::zeros(diff.rows,diff.cols,CV_8UC1);

    for (int i = 0; i < max_points_per_frame; i++) {
        Point mint;
        Point maxt;
        double min, max;
        minMaxLoc(diff, &min, &max, &mint, &maxt);

        uint8_t motion_noise = motion_noise_mapL.at<uint8_t>(maxt.y,maxt.x);

        int motion_thresh_tmp = motion_thresh;
        if (_mode == mode_locate_drone) {
            motion_thresh_tmp = motion_thresh + dparams.drone_blink_strength;
            motion_noise = 0; // motion noise calib is done during blink detection. To prevent interference do not use the motion_noise motion noise
        }

        bool thresh_res = max > motion_noise+motion_thresh_tmp;
        if (!thresh_res) { // specifcally for the insect tracker, check if there is an increased chance in this area
            for (auto trkr : _trackers) {
                tracking::ImagePredictItem ipi = trkr->image_predict_item();
                if (trkr->tracking() && trkr->type() == tt_insect) {
                    cv::Point2f d;
                    d.x = ipi.pt().x - maxt.x*pparams.imscalef;
                    d.y = ipi.pt().y - maxt.y*pparams.imscalef;
                    float dist = norm(d);
                    float chance = 1;
                    if (ipi.valid &&ipi.pixel_max < 1.5f * motion_thresh_tmp)
                        chance +=chance_multiplier_pixel_max;
                    if (dist < roi_radius*pparams.imscalef) {
                        chance += chance_multiplier_dist;
                    }
                    thresh_res = static_cast<uint8_t>(max) > motion_noise+(motion_thresh_tmp/chance);
                }
            }
        }

        if (thresh_res) {
            find_cog_and_remove(maxt,max,diff,enable_insect_drone_split,drn_ins_split_thresh,motion_noise_mapL);
        } else {

            int tmp_mt_noise = motion_noise;
            if (max < motion_noise+motion_thresh_tmp)
                tmp_mt_noise = 0;
            if  (static_cast<uint8_t>(max) <= tmp_mt_noise+(motion_thresh_tmp/chance_multiplier_total))
                break; // done searching for maxima, they are too small now
            else
                cv::circle(diff, maxt, roi_radius, Scalar(0), cv::FILLED);
        }
    }
}
void TrackerManager::find_cog_and_remove(cv::Point maxt, double max, cv::Mat diff,bool enable_insect_drone_split, float drn_ins_split_thresh,cv::Mat motion_noise_mapL) {
    //get the Rect containing the max movement:
    Rect rect(maxt.x-roi_radius, maxt.y-roi_radius, roi_radius*2,roi_radius*2);
    if (rect.x < 0)
        rect.x = 0;
    else if (rect.x+rect.width >= diff.cols)
        rect.x =  diff.cols -rect.width;
    if (rect.y < 0)
        rect.y = 0;
    else if (rect.y+rect.height >= diff.rows)
        rect.y =  diff.rows -rect.height;
    Mat roi(diff,rect); // so, this is the cut out around the max point

    // make a black mask, same size:
    Mat mask = Mat::zeros(roi.size(), roi.type());
    // with a white, filled circle in it:
    cv::circle(mask, Point(roi_radius,roi_radius), roi_radius, 32765, -1);
    // combine roi & mask:
    Mat cropped = roi & mask;

    //threshold to get only pixels that are heigher then the motion noise
    mask = cropped > motion_noise_mapL(rect) + motion_thresh;

    Moments mo = moments(mask,true);
    Point2f COG = Point2f(static_cast<float>(mo.m10) / static_cast<float>(mo.m00), static_cast<float>(mo.m01) / static_cast<float>(mo.m00));

    if (COG.x != COG.x) { // no valid COG, lower threshold
        mask = cropped > motion_noise_mapL(rect) + 1;
        mo = moments(mask,true);
        COG = Point2f(static_cast<float>(mo.m10) / static_cast<float>(mo.m00), static_cast<float>(mo.m01) / static_cast<float>(mo.m00));
    }

    if (COG.x != COG.x) { // still no joy, try yet another way. This shouldn't happen to often though:
        Scalar avg = mean(cropped);
        Scalar avg_bkg =mean(motion_noise_mapL(rect));
        //blur, to filter out noise
        cv::GaussianBlur(cropped,cropped,Size(5,5),0);

        int mask_thresh = (max-avg_bkg(0)) * 0.1+avg_bkg(0);
        if (enable_insect_drone_split)
            mask_thresh = drn_ins_split_thresh +  static_cast<float>(avg_bkg(0));

        Point dummy;
        double min;
        minMaxLoc(cropped, &min, &max, &dummy, &dummy);
        mask_thresh = (max-avg(0)) * 0.6; // use a much lower threshold because the noise was blurred away
        mask = cropped > mask_thresh;
        mo = moments(mask,true);
        COG = Point2f(static_cast<float>(mo.m10) / static_cast<float>(mo.m00), static_cast<float>(mo.m01) / static_cast<float>(mo.m00));
    }

    Mat viz;
    if (enable_viz_max_points) {
        Rect rect_unscaled(rect.x*pparams.imscalef,rect.y*pparams.imscalef,rect.width*pparams.imscalef,rect.height*pparams.imscalef);
        cv::Mat frameL_roi = _visdat->frameL(rect_unscaled);
        cv::Mat frameL_small_roi;
        cv::resize(frameL_roi,frameL_small_roi,cv::Size(frameL_roi.cols/pparams.imscalef,frameL_roi.rows/pparams.imscalef));

        viz = create_row_image({roi,mask,frameL_small_roi,motion_noise_mapL(rect)*10},CV_8UC1,viz_max_points_resizef);
        cvtColor(viz,viz,cv::COLOR_GRAY2BGR);
        if (enable_insect_drone_split)
            putText(viz,"i-d",Point(0, viz.rows-13),FONT_HERSHEY_SIMPLEX,0.3,Scalar(255,255,255));
    }
    bool viz_pushed = false;

    // relative it back to the _approx frame
    COG.x += rect.x;
    COG.y += rect.y;

    bool single_blob = true;
    bool COG_is_nan = false;

    if (enable_insect_drone_split && target_insecttracker()) {

        float dist_to_predict = normf(_dtrkr->image_predict_item().pt() - COG*pparams.imscalef);
        if (dist_to_predict < 20) {

            //check if the blob may be multiple blobs,
            vector<vector<Point>> contours;
            findContours(mask,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE); // If necessary, we could do this on the full resolution image...?
            if (contours.size()==1 && enable_insect_drone_split) { // try another threshold value, sometimes we get lucky
                drn_ins_split_thresh = target_insecttracker()->image_predict_item().pixel_max*0.3f;
                Scalar avg_bkg =mean(motion_noise_mapL(rect));
                mask = cropped > drn_ins_split_thresh + static_cast<float>(avg_bkg(0));
                findContours(mask,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE);
            }

            if (contours.size()>1) {
                //ok, definetely multiple blobs. Split them, and find the COG for each.
                single_blob = false;
                for (uint j = 0; j< contours.size(); j++) {
                    Point2f COG2;
                    if (contours.at(j).size() < 3) { // to prevent COG nan
                        COG2 = contours.at(j).at(0);
                    } else {
                        Moments mo2 = moments(contours.at(j),true);
                        COG2 = Point2f(static_cast<float>(mo2.m10) / static_cast<float>(mo2.m00), static_cast<float>(mo2.m01) / static_cast<float>(mo2.m00));
                    }

                    if (COG2.x == COG2.x) {// if not nan
                        cv::Point2f center;
                        float radius;
                        cv::minEnclosingCircle(contours.at(j),center,radius);
                        if (enable_viz_max_points) {
                            cv::Mat viz2 = viz.clone();
                            vizs_maxs.push_back(viz2);
                            cv::Point2f COG2_viz = COG2*viz_max_points_resizef;
                            cv::circle(viz2,COG2_viz,1,Scalar(0,0,255),1); //COG
                            cv::circle(viz2,COG2_viz,roi_radius*viz_max_points_resizef,Scalar(0,0,255),1);  // remove radius
                            cv::circle(viz2,COG2_viz,radius*viz_max_points_resizef,Scalar(0,255,0),1);  // blob radius
                            putText(viz2,to_string_with_precision(COG2_viz.y,0),COG2_viz,FONT_HERSHEY_SIMPLEX,0.3,Scalar(100,0,255));
                            putText(viz2,std::to_string(vizs_maxs.size()) + ", " + std::to_string(j),Point(0, 13),FONT_HERSHEY_SIMPLEX,0.3,Scalar(255,255,0));
                            viz_pushed = true;
                        }
                        // relative COG back to the _approx frame, and save it:
                        COG2.x += rect.x;
                        COG2.y += rect.y;
                        uchar px_max = diff.at<uchar>(COG2);
                        _blobs.push_back(tracking::BlobProps(COG2,maxt,radius*2,px_max,mask,_visdat->is_in_overexposed_area(COG2)));

                        //remove this COG from the ROI:
                        cv::circle(diff, COG2, roi_radius, Scalar(0), cv::FILLED);
                    } else {
                        COG_is_nan = true;
                    }
                }
            }
        }
    }
    if (!viz_pushed &&  enable_viz_max_points) {
        vizs_maxs.push_back(viz);
        putText(viz,std::to_string(vizs_maxs.size()-1),Point(0, 13),FONT_HERSHEY_SIMPLEX,0.3,Scalar(255,255,255));
    }
    if (single_blob) { // we could not split this blob, so we can use the original COG
        float size = sqrtf(mo.m00/M_PI)*2.f; // assuming a circular shape
        if (COG.x == COG.x) { // if not nan
            _blobs.push_back(tracking::BlobProps(COG, maxt, size,max, mask,_visdat->is_in_overexposed_area(COG)));
            if (enable_viz_max_points) {
                Point2f tmpCOG;
                tmpCOG.x = COG.x - rect.x;
                tmpCOG.y = COG.y - rect.y;
                tmpCOG *= viz_max_points_resizef;
                cv::circle(viz,tmpCOG,1,Scalar(0,0,255),1);
                cv::circle(viz,tmpCOG,roi_radius*viz_max_points_resizef,Scalar(0,0,255),1);  // remove radius
                cv::circle(viz,tmpCOG,0.5f*size*viz_max_points_resizef,Scalar(0,255,0),1);  // blob radius
                putText(viz,to_string_with_precision(tmpCOG.y,0),tmpCOG,FONT_HERSHEY_SIMPLEX,0.3,Scalar(255,0,0));
            }
            //remove this COG from the ROI:
            cv::circle(diff, COG, roi_radius, Scalar(0), cv::FILLED);
        } else {
            COG_is_nan = true;
            if (enable_insect_drone_split) {
                //TODO: below is double code, streamline
                for (auto trkr : _trackers) {
                    tracking::ImagePredictItem ipi = trkr->image_predict_item();
                    cv::Point2f d;
                    d.x = ipi.pt().x - maxt.x*pparams.imscalef;
                    d.y = ipi.pt().y - maxt.y*pparams.imscalef;
                    float dist = norm(d);
                    if (dist < roi_radius*pparams.imscalef) {
                        _blobs.push_back(tracking::BlobProps(maxt, maxt, 1,max, mask,_visdat->is_in_overexposed_area(maxt)));
                        if (enable_viz_max_points) {
                            Point2f tmpCOG;
                            tmpCOG.x = maxt.x - rect.x;
                            tmpCOG.y = maxt.y - rect.y;
                            tmpCOG *= viz_max_points_resizef;
                            cv::circle(viz,tmpCOG,1,Scalar(0,0,255),1);
                            cv::circle(viz,tmpCOG,roi_radius*viz_max_points_resizef,Scalar(0,0,255),1);  // remove radius
                            cv::circle(viz,tmpCOG,0.5f*size*viz_max_points_resizef,Scalar(0,255,0),1);  // blob radius
                            putText(viz,"maxt " + to_string_with_precision(tmpCOG.y,0),tmpCOG,FONT_HERSHEY_SIMPLEX,0.3,Scalar(255,0,0));
                        }
                        //remove this COG from the ROI:
                        cv::circle(diff, maxt, roi_radius, Scalar(0), cv::FILLED);
                        COG_is_nan = false;
                        break;
                    }
                }
            }
        }
    }
    if (COG_is_nan)  //remove the actual maximum from the ROI if the COG algorithm failed:
        cv::circle(diff, maxt, roi_radius, Scalar(0), cv::FILLED);
}

void TrackerManager::collect_static_ignores() {
    tracking::IgnoreBlob landingspot;
    for ( auto & trkr : _trackers) {
        collect_static_ignores(trkr);
    }
}
void TrackerManager::collect_static_ignores(ItemTracker *trkr1) {
    tracking::IgnoreBlob landingspot;
    std::vector<tracking::IgnoreBlob> ignores;
    for ( auto & trkr2 : _trackers) {
        if (trkr1->uid()!=trkr2->uid()) {
            for (auto & ign : trkr2->ignores_for_other_trkrs) {
                ignores.push_back(ign);
                if (ign.ignore_type == tracking::IgnoreBlob::takeoff_spot) {
                    landingspot = ign;
                }
            }
        }
        trkr1->ignores_for_me = ignores;
    }
}
void TrackerManager::erase_dissipated_fps(double time) {
    false_positives.erase(
        std::remove_if(false_positives.begin(), false_positives.end(),
    [&](auto& fp) {return time - fp.last_seen_time > std::clamp(fp.detection_count * 10,1,60);} ),
    false_positives.end());
}

void TrackerManager::match_blobs_to_trackers(bool drone_is_active, double time) {

    for (auto trkr : _trackers)
        trkr->all_blobs(_blobs);

    std::vector<ProcessedBlob> pbs;
    if (_blobs.size()>0) {
        prep_blobs(&pbs,time);
        match_existing_trackers(&pbs,drone_is_active,time);
        check_match_conflicts(&pbs,time);
        match_trackers_that_lost(&pbs,drone_is_active,time);
        flag_used_static_ignores(&pbs);
        create_new_insect_trackers(&pbs,time);
    }

    draw_viz(&pbs,time);

    for (auto trkr : _trackers)
        trkr->invalidize();
}
void TrackerManager::prep_blobs(std::vector<ProcessedBlob> *pbs,double time) {
    //init processed blob list and check on fp's
    uint id_cnt = 0;
    for (auto & blob : _blobs) {
        pbs->push_back(ProcessedBlob(&blob,id_cnt));
        id_cnt++;
        for (auto & fp : false_positives) {
            float dist = normf(fp.im_pt-blob.pt()*pparams.imscalef);
            if (dist < fp.im_size) {
                blob.false_positive = true;
                fp.last_seen_time = time;
                fp.detection_count++;
            }
        }
    }
}
void TrackerManager::match_existing_trackers(std::vector<ProcessedBlob> *pbs,bool drone_is_active, double time) {
    //first check if there are trackers that were already tracking something, which prediction matches a new keypoint
    for (auto trkr : _trackers) {
        float best_trkr_score = INFINITY;
        ProcessedBlob *best_blob;

        if (tracker_active(trkr,drone_is_active) && trkr->tracking()) {
            for (auto &blob : *pbs) {
                auto *props = blob.props;
                bool in_ignore_zone = trkr->check_ignore_blobs(props);
                if (in_ignore_zone)
                    blob.ignored = true;
                if (!in_ignore_zone) {
                    float score  = trkr->score(props);
                    if (score < best_trkr_score) {
                        best_trkr_score = score;
                        best_blob = &blob;
                    }
                    if (enable_viz_max_points && trkr->viz_id() < 6) {
                        cv::Mat viz = vizs_maxs.at(blob.id);
                        cv::Point2i pt(viz.cols/4 + 2,viz.rows - 14*(trkr->viz_id()+1));
                        putText(viz,"#" + std::to_string(trkr->viz_id()) + ":" + to_string_with_precision(score,2) + "/" + to_string_with_precision(trkr->predicted_score(),2),pt,FONT_HERSHEY_SIMPLEX,0.3,tracker_color(trkr));
                    }
                } else if (enable_viz_max_points) {
                    cv::Mat viz = vizs_maxs.at(blob.id);
                    cv::Point2i pt(viz.cols/4+2,viz.rows -6-14*(trkr->viz_id()+1));
                    putText(viz,"Ign.",pt,FONT_HERSHEY_SIMPLEX,0.3,tracker_color(trkr));
                }
            }
            if (best_trkr_score < trkr->score_threshold() ) {
                best_blob->trackers.push_back(trkr);
                trkr->calc_world_item(best_blob->props,time);
                tracking::ImageItem iti(*(best_blob->props),_visdat->frame_id,best_trkr_score,best_blob->id);
                tracking::WorldItem w(iti,best_blob->props->world_props);
                trkr->world_item(w);
            }
        }
    }
}
void TrackerManager::check_match_conflicts(std::vector<ProcessedBlob> *pbs,double time) {
    //check if there are blobs having multiple trackers attached
    for (auto &blob_i : *pbs) {
        if (blob_i.trackers.size()> 1) {
            //see if we have a drone / insect tracker pair
            DroneTracker *dtrkr;
            InsectTracker *itrkr;
            bool drn_trkr_fnd = false;
            for (auto trkr : blob_i.trackers) {
                if (trkr->type() == tt_drone) {
                    dtrkr = static_cast<DroneTracker *>(trkr);
                    drn_trkr_fnd = true;
                    break;
                }
            }

            if (drn_trkr_fnd) {
                auto props_i = blob_i.props;
                for (auto trkr : blob_i.trackers) {
                    if (trkr->type() == tt_insect) {
                        itrkr = static_cast<InsectTracker*>(trkr);

                        //so, there should be another blob close to the currently one used.
                        //if there is, the small one is prolly the moth...
                        bool conflict_resolved = false;
                        for (auto &blob_j : *pbs) {
                            if (blob_i.id != blob_j.id && !blob_j.tracked()) {
                                auto props_j = blob_j.props;
                                float dist = cv::norm(blob_i.pt()-blob_j.pt());
                                if (dist < 2.f* (blob_i.size() + blob_j.size())) {
                                    if(blob_i.size() > blob_j.size()) {
                                        dtrkr->calc_world_item(props_i,time);
                                        tracking::WorldItem wi(tracking::ImageItem(*props_i,_visdat->frame_id,0,blob_i.id),props_i->world_props);
                                        dtrkr->world_item(wi);

                                        itrkr->calc_world_item(props_j,time);
                                        tracking::WorldItem wj(tracking::ImageItem(*props_j,_visdat->frame_id,0,blob_j.id),props_j->world_props);
                                        itrkr->world_item(wj);

                                        blob_i.trackers.clear();
                                        blob_j.trackers.clear();
                                        blob_i.trackers.push_back(dtrkr);
                                        blob_j.trackers.push_back(itrkr);
                                    } else {
                                        itrkr->calc_world_item(props_i,time);
                                        tracking::WorldItem wi(tracking::ImageItem(*props_i,_visdat->frame_id,0,blob_i.id),props_i->world_props);
                                        itrkr->world_item(wi);
                                        dtrkr->calc_world_item(props_j,time);
                                        tracking::WorldItem wj(tracking::ImageItem(*props_j,_visdat->frame_id,0,blob_j.id),props_j->world_props);
                                        dtrkr->world_item(wj);

                                        blob_i.trackers.clear();
                                        blob_j.trackers.clear();
                                        blob_j.trackers.push_back(dtrkr);
                                        blob_i.trackers.push_back(itrkr);
                                    }
                                    conflict_resolved = true;
                                    break;
                                }
                            }
                        }

                        if (!conflict_resolved) {
                            dtrkr->blobs_are_fused();
                            itrkr->blobs_are_fused();

                            //TODO: do something sensible in cases 1,2,4:
                            // hmm, apparantely there is no blob close by, so now there are a few possibilities:
                            //1. The insect is lost (e.g. too far, too small, into the flowers)
                            //2. The drone is lost (e.g. crashed)
                            //3. The insect and drone are too close to eachother to distinguish
                            //4. We have a kill :) In the last few cases this resulted in a confetti storm. Maybe this can be detected.
                        }
                    }
                }
            } else { //for some reason multiple insect trackers got fused. This is quite unlikely, so just select the one with the best score:
                ItemTracker *best_tracker;
                float best_score = INFINITY;
                for (auto trkr : blob_i.trackers) {
                    if (trkr->image_item().score < best_score) {
                        best_score = trkr->image_item().score;
                        best_tracker = trkr;
                    }
                }
                for (auto trkr : blob_i.trackers) {
                    if (trkr->uid() != best_tracker->uid()) {
                        WorldItem w;
                        trkr->world_item(w);
                    }
                }
                blob_i.trackers.clear();
                blob_i.trackers.push_back(best_tracker);
            }
        }
    }

}
void TrackerManager::match_trackers_that_lost(std::vector<ProcessedBlob> *pbs,bool drone_is_active, double time) {
    //see if there are trackers that are not tracking yet and if there are untracked points left, which can be bound together.
    for (auto trkr : _trackers) {
        if (!trkr->tracking()) {
            if (tracker_active(trkr, drone_is_active)) {
                for (auto &blob : *pbs) {
                    if (!blob.tracked() && (!blob.props->in_overexposed_area || trkr->type() == tt_blink) && (!blob.props->false_positive || trkr->type() == tt_blink)) {

                        //check against static ignore points
                        auto props = blob.props;
                        bool in_im_ignore_zone = trkr->check_ignore_blobs(props);
                        if (in_im_ignore_zone)
                            blob.ignored = true;

                        if (!in_im_ignore_zone) {
                            //Check if this blob may be some residual drone motion
                            //If the insect was lost, it is unlikely that it or another insect pops up
                            //very close to the drone. So ignore it.
                            if ((trkr->type() == tt_drone) && _dtrkr->tracking()) {
                                float drone_score = _dtrkr->score(props);
                                if (drone_score > _dtrkr->score_threshold())
                                    in_im_ignore_zone = true;
                            }
                        }

                        if (!in_im_ignore_zone) {
                            //The tracker has lost the item, or is still initializing.
                            //there is no valid prediction, the score is therefor as low as possible
                            trkr->calc_world_item(props,time);
                            tracking::WorldItem w(tracking::ImageItem(*props,_visdat->frame_id,0,blob.id),props->world_props);
                            if (w.valid) {
                                trkr->world_item(w);
                                blob.trackers.push_back(trkr);

                                if (enable_viz_max_points) {
                                    cv::Mat viz = vizs_maxs.at(blob.id);
                                    cv::Point2i pt(viz.cols/4+2,viz.rows -6-14*(trkr->viz_id()+1));
                                    putText(viz,"New",pt,FONT_HERSHEY_SIMPLEX,0.3,tracker_color(trkr));
                                }

                                // There may be other interesting blobs to consider but since there is no
                                // history available at this point, its hard to calculate which one would
                                // be better. So just pick the first one...:
                                break;
                            }
                        } else {
                            if (enable_viz_max_points) {
                                cv::Mat viz = vizs_maxs.at(blob.id);
                                cv::Point2i pt(viz.cols/4+2,viz.rows-6-14*(trkr->viz_id()+1));
                                putText(viz,"Ign.",pt,FONT_HERSHEY_SIMPLEX,0.3,tracker_color(trkr));
                            }
                        }
                    } else {
                        if (enable_viz_max_points) {
                            cv::Mat viz = vizs_maxs.at(blob.id);
                            cv::Point2i pt(viz.cols/4+2,viz.rows -6-14*(trkr->viz_id()+1));
                            putText(viz,"Trkd.",pt,FONT_HERSHEY_SIMPLEX,0.3,tracker_color(trkr));
                        }
                    }
                }
            }
        }
    }
}
void TrackerManager::flag_used_static_ignores(std::vector<ProcessedBlob> *pbs) {
    //see if there are static ignore points that are detected. If so set was_used flag
    for (auto trkr : _trackers) {
        for (auto blob : *pbs) {
            for (auto& ignore : trkr->ignores_for_other_trkrs) {
                float dist_ignore = normf(ignore.p - blob.pt());
                if (dist_ignore < blob.size() + ignore.radius ) {
                    ignore.was_used = true;
                }
            }
        }
    }
}
void TrackerManager::create_new_insect_trackers(std::vector<ProcessedBlob> *pbs, double time) {
    //see if there are still blobs left untracked, create new trackers for them
    for (auto &blob : *pbs) {
        auto props = blob.props;
        if (!blob.tracked() && (!props->in_overexposed_area || _mode == mode_locate_drone) && (!props->false_positive || _mode == mode_locate_drone) && !blob.ignored && _trackers.size() < 30) { // if so, start tracking it!
            if (_mode == mode_locate_drone) {

                bool too_close = false;
                for (auto btrkr : _trackers) {
                    if (btrkr->type() == tt_blink) {
                        if (normf(btrkr->image_item().pt() - props->pt_unscaled())  < roi_radius*pparams.imscalef) {
                            too_close = true;
                        }
                    }
                }

                if (!too_close) {
                    BlinkTracker  *bt;
                    bt = new BlinkTracker();
                    bt->init(next_blinktrkr_id,_visdat,motion_thresh, _trackers.size());
                    bt->calc_world_item(props,time);
                    bool ignore = true;
                    if (props->world_props.valid) {
                        collect_static_ignores(bt);
                        ignore = bt->check_ignore_blobs(props);
                    }
                    if (!ignore) {
                        bt->init_logger();
                        next_blinktrkr_id++;
                        tracking::WorldItem w(tracking::ImageItem(*props,_visdat->frame_id,0,blob.id),props->world_props);
                        bt->world_item(w);
                        _trackers.push_back( bt);
                        blob.trackers.push_back(bt);
                    } else {
                        bt->close();
                        delete bt;
                    }
                }
            } else if (_mode != mode_idle && _mode != mode_drone_only) {
                float im_dist_to_drone;
                if (_dtrkr->tracking()) {
                    im_dist_to_drone = normf(_dtrkr->image_item().pt()-props->pt_unscaled());
                } else {
                    if (_dtrkr->takeoff_location_valid())
                        im_dist_to_drone = normf(_dtrkr->takeoff_im_location() - props->pt_unscaled());
                    else
                        im_dist_to_drone = INFINITY;
                }
                if (im_dist_to_drone > InsectTracker::new_tracker_drone_ignore_zone_size_im) {
                    InsectTracker *it;
                    it = new InsectTracker();
                    it->init(next_insecttrkr_id,_visdat,motion_thresh,_trackers.size());
                    it->calc_world_item(props,time);
                    if (!props->world_props.bkg_check_ok) {
                        tracking::WorldItem w(tracking::ImageItem(*props,_visdat->frame_id,-1,blob.id),props->world_props);
                        false_positives.push_back(FalsePositive(w,fp_bkg,time));
                    }

                    bool delete_it = true;
                    bool ignore = true;
                    if (props->world_props.valid) {
                        collect_static_ignores(it);
                        ignore = it->check_ignore_blobs(props);
                        blob.ignored = ignore;
                    }
                    if (!ignore) {
                        //ignore a region around the drone (or take off location)
                        float world_dist_to_drone;
                        if (_dtrkr->tracking())
                            world_dist_to_drone = normf(_dtrkr->world_item().pt- props->world_props.pt());
                        else
                            world_dist_to_drone = normf(_dtrkr->takeoff_location() - props->world_props.pt());

                        if (world_dist_to_drone > InsectTracker::new_tracker_drone_ignore_zone_size_world && im_dist_to_drone > InsectTracker::new_tracker_drone_ignore_zone_size_im) {
                            it->init_logger();
                            next_insecttrkr_id++;
                            tracking::WorldItem w(tracking::ImageItem(*props,_visdat->frame_id,0,blob.id),props->world_props);
                            it->world_item(w);
                            _trackers.push_back(it);
                            blob.trackers.push_back(it);
                            delete_it = false;
                        } else {
                            blob.ignored = true;
                        }
                    }
                    if (delete_it) {
                        it->close();
                        delete it;
                    }
                } else {
                    blob.ignored = true;
                }

            }
        }
    }
}

void TrackerManager::update_trackers(double time,long long frame_number, bool drone_is_active) {
    //perform all tracker update functions or logging placeholders, also delete old trackers
    for (uint ii=_trackers.size(); ii != 0; ii--) { // reverse because deleting from this list.
        uint i = ii-1;
        if (_trackers.at(i)->delete_me()) {
            ItemTracker *trkr = _trackers.at(i);
            if (trkr->type() == tt_insect ) {
                InsectTracker *itrkr = static_cast<InsectTracker *>(_trackers.at(i));
                auto fpt = itrkr->false_positive();
                if (fpt)
                    false_positives.push_back(FalsePositive(itrkr->track_history.begin()->world_item,fpt,time));
            }
            trkr->close();
            _trackers.erase(_trackers.begin() + i);
            delete trkr;
        } else if(_trackers.at(i)->type() == tt_drone) {
            DroneTracker *dtrkr = static_cast<DroneTracker *>(_trackers.at(i));
            dtrkr->update(time,tracker_active(dtrkr,drone_is_active));
        } else if (_trackers.at(i)->type() == tt_insect) {
            InsectTracker *itrkr = static_cast<InsectTracker *>(_trackers.at(i));
            switch (_mode) {
            case mode_idle: {
                itrkr->append_log(time,frame_number); // write dummy data
                break;
            } case mode_locate_drone: {
                itrkr->append_log(time,frame_number); // write dummy data
                break;
            } case mode_wait_for_insect: {
                itrkr->update(time);
                break;
            } case mode_hunt: {
                itrkr->update(time);
                break;
            } case mode_drone_only: {
                itrkr->append_log(time,frame_number); // write dummy data
                break;
            }
            }
        } else if(_trackers.at(i)->type() == tt_replay) {
            ReplayTracker *rtrkr = static_cast<ReplayTracker *>(_trackers.at(i));
            rtrkr->update(time);
        } else if(_trackers.at(i)->type() == tt_virtualmoth) {
            VirtualmothTracker*vtrkr = static_cast<VirtualmothTracker*>(_trackers.at(i));
            vtrkr->update(time);
        } else if (_trackers.at(i)->type() == tt_blink) {
            BlinkTracker *btrkr = static_cast<BlinkTracker *>(_trackers.at(i));
            btrkr->update(time);
            if (_mode == mode_locate_drone) {
                if (btrkr->state() == BlinkTracker::bds_found) {
                    _dtrkr->set_landing_location(btrkr->image_item().pt(),btrkr->world_item().iti.disparity,btrkr->smoothed_size_image(),btrkr->world_item().pt);
                    _camview->p0_bottom_plane(_dtrkr->takeoff_location().y, true);
                    _mode = mode_idle;
                }
            } else if (btrkr->ignores_for_other_trkrs.size() == 0) {
                if (btrkr->state() != BlinkTracker::bds_found) {
                    double v = 0;
                    if (btrkr->last_track_data().vel_valid)
                        v = norm(btrkr->last_track_data().vel());
                    if (v < 0.2)
                        _visdat->delete_from_motion_map(btrkr->image_item().pt(),btrkr->image_item().disparity,btrkr->image_item().size+5,1);
                }

                btrkr->close();
                _trackers.erase(_trackers.begin() + i);
                delete btrkr;
            }
        }
    }
}

void TrackerManager::prep_vizs() {
    if (enable_viz_diff) {
        cv::cvtColor(_visdat->diffL*10,diff_viz,cv::COLOR_GRAY2BGR);
    }

    reset_trkr_viz_ids();
}
void TrackerManager::draw_viz(std::vector<ProcessedBlob> *pbs, double time) {
    if (enable_viz_diff) {
        for (auto blob : *pbs) {
            std::string s = std::to_string(blob.id);
            for (auto trkr : blob.trackers) {
                if (trkr->type() == tt_insect) {
                    InsectTracker *itrkr = static_cast<InsectTracker*>(trkr);
                    s = s + "->" + std::to_string(itrkr->insect_trkr_id());
                }
            }
            cv::Point target_viz_p = (blob.props->pt_unscaled() + cv::Point2f(10,-10));
            putText(diff_viz,blob.prefix() +  s,target_viz_p,FONT_HERSHEY_SIMPLEX,0.3,blob.color(),2);
            cv::line(diff_viz,target_viz_p,blob.props->pt_unscaled(),blob.color());

        }
        for (auto trkr : replaytrackers()) {
            cv::Point target_viz_p = (trkr->image_item().pt() + cv::Point2f(10,-10));
            putText(diff_viz," r",target_viz_p,FONT_HERSHEY_SIMPLEX,0.3,cv::Scalar(0,0,180),2);
            cv::line(diff_viz,target_viz_p,trkr->image_item().pt(),cv::Scalar(0,0,180));
        }
        for (auto trkr : virtualmothtrackers()) {
            cv::Point target_viz_p = (trkr->image_item().pt() + cv::Point2f(10,-10));
            putText(diff_viz," v",target_viz_p,FONT_HERSHEY_SIMPLEX,0.3,cv::Scalar(0,0,180),2);
            cv::line(diff_viz,target_viz_p,trkr->image_item().pt(),cv::Scalar(0,0,180));
        }

        auto blue =cv::Scalar(255,0,0);
        for (auto fp : false_positives) {
            auto p_center = fp.im_pt;
            cv::circle(diff_viz,p_center,fp.im_size,blue);
            cv::putText(diff_viz,to_string_with_precision(time-fp.last_seen_time,1) + ", #" + std::to_string(fp.detection_count),p_center+cv::Point2f(10,0),FONT_HERSHEY_SIMPLEX,0.3,blue,2);
        }
    }
    if (enable_viz_max_points) {
        for (auto blob : *pbs) {
            auto wblob = blob.props->world_props;
            std::string msg_str = std::to_string(blob.id);
            cv::Mat viz = vizs_maxs.at(blob.id);
            if (!wblob.valid && !blob.ignored) {
                if (!wblob.im_pos_ok)
                    msg_str = msg_str + "pre.";
                else {
                    if (!wblob.disparity_in_range)
                        msg_str = msg_str + " dsp.";
                    if (!wblob.bkg_check_ok)
                        msg_str = msg_str + " bkg.";

                    if (!wblob.radius_in_range)
                        msg_str = msg_str + " r.";
                    if (blob.props->ignores.size()>0)
                        msg_str = msg_str + " ign.";
                    if (wblob.takeoff_reject)
                        msg_str = msg_str + " tko.";
                }
                putText(viz,msg_str,cv::Point2i(viz.cols/4+2,viz.rows - 20),FONT_HERSHEY_SIMPLEX,0.3,blob.color());
            }
            if (wblob.valid) {
                std::string coor_str = "[" + to_string_with_precision(wblob.x,2) + ", " + to_string_with_precision(wblob.y,2) + ", " + to_string_with_precision(wblob.z,2) +"]";
                putText(viz,coor_str,cv::Point2i(viz.cols/4+2,viz.rows - 10),FONT_HERSHEY_SIMPLEX,0.3,blob.color());
            }
        }
    }
}
void TrackerManager::finish_vizs() {
    if (enable_viz_max_points && vizs_maxs.size()>0)
        viz_max_points = create_column_image(vizs_maxs,CV_8UC3,1);
    else if(enable_viz_max_points)
        viz_max_points = cv::Mat::zeros(5,100,CV_8UC3);
    if (enable_viz_diff)
        diff_viz_buf = diff_viz.clone();;
}
void TrackerManager::reset_trkr_viz_ids() {
    int16_t cnt = 0;
    for (auto trkr : _trackers)
        trkr->viz_id(cnt++);
}

bool TrackerManager::tracker_active(ItemTracker *trkr, bool drone_is_active) {
    if (_mode == mode_idle)
        return false;
    else if (trkr->type() == tt_drone && (_mode == mode_locate_drone || !drone_is_active)) {
        return false;
    } else if (trkr->type() == tt_insect && (_mode == mode_locate_drone || _mode == mode_drone_only )) {
        return false;
    } else if (trkr->type() == tt_blink && (_mode != mode_locate_drone)) {
        return false;
    } else if (trkr->type() == tt_replay) {
        return false;
    } else if (trkr->type() == tt_virtualmoth) {
        return false;
    }
    return true;
}

std::vector<BlinkTracker *> TrackerManager::blinktrackers() {
    std::vector<BlinkTracker *> res;
    for (auto trkr : _trackers) {
        if (trkr->type() == tt_blink) {
            res.push_back(static_cast<BlinkTracker *>(trkr));
        }
    }
    return res;
}
std::vector<ReplayTracker *> TrackerManager::replaytrackers() {
    std::vector<ReplayTracker *> res;
    for (auto trkr : _trackers) {
        if (trkr->type() == tt_replay) {
            res.push_back(static_cast<ReplayTracker *>(trkr));
        }
    }
    return res;
}
std::vector<VirtualmothTracker *> TrackerManager::virtualmothtrackers() {
    std::vector<VirtualmothTracker *> res;
    for (auto trkr : _trackers) {
        if (trkr->type() == tt_virtualmoth) {
            res.push_back(static_cast<VirtualmothTracker *>(trkr));
        }
    }
    return res;
}
std::vector<InsectTracker *> TrackerManager::insecttrackers() {
    std::vector<InsectTracker *> res;
    for (auto trkr : _trackers) {
        if (trkr->type() == tt_insect) {
            res.push_back(static_cast<InsectTracker *>(trkr));
        }
    }
    //should do the same:
    //std::copy_if (_trackers.begin(), _trackers.end(), std::back_inserter(res), [](ItemTracker *trkr){return (trkr->type == tt_insect);} );
    return res;
}
std::vector<ItemTracker *> TrackerManager::all_target_trackers() {
    std::vector<ItemTracker *> res;
    for (auto trkr : _trackers) {
        if (trkr->type() == tt_insect || trkr->type() == tt_replay || trkr->type() == tt_virtualmoth) {
            res.push_back(trkr);
        }
    }
    return res;
}

TrackData TrackerManager::target_last_trackdata() {
    if (target_insecttracker())
        return target_insecttracker()->last_track_data();
    else {
        TrackData d;
        return d;
    }
}
bool TrackerManager::tracking_a_target() {
    if (target_insecttracker())
        return target_insecttracker()->tracking();
    else
        return false;
}
double TrackerManager::target_last_detection() {
    if (target_insecttracker()) {
        return target_insecttracker()->last_detection();
    } else
        return 0;
}

InsectTracker *TrackerManager::target_insecttracker() {

    if (target_insecttracker_frameid_updated == _visdat->frame_id)
        return _target_insecttracker;

    if (target_insecttracker_frameid_updated>0)
        if (_target_insecttracker)
            if (_target_insecttracker->tracking()) { // #599
                target_insecttracker_frameid_updated = _visdat->frame_id;
                return _target_insecttracker;
            }

    float best_dist = INFINITY;
    InsectTracker *best_itrkr = NULL;

    cv::Point3f current_drone_pos = _dtrkr->last_track_data().pos();
    if (_dtrkr->last_track_data().pos_valid) {
        if (_dtrkr->takeoff_location_valid())
            current_drone_pos = _dtrkr->takeoff_location();
        else
            current_drone_pos = cv::Point3f(0,-1.5,-1.5);
    }

    for (auto trkr : _trackers) {
        if (trkr->tracking() ) {
            if (trkr->type() == tt_insect || trkr->type() == tt_replay || trkr->type() == tt_virtualmoth) {
                float dist = normf(current_drone_pos- trkr->last_track_data().pos());
                if (best_dist > dist) {
                    best_dist = dist;
                    best_itrkr = static_cast<InsectTracker *>(trkr);
                }
            }
        }
    }

    target_insecttracker_frameid_updated = _visdat->frame_id;
    _target_insecttracker = best_itrkr;
    return best_itrkr;
}
std::tuple<bool, BlinkTracker *> TrackerManager::blinktracker_best() {
    BlinkTracker::blinking_drone_states best_state = BlinkTracker::bds_searching;
    BlinkTracker *best_btrkr;
    bool valid = false;
    for (auto trkr : blinktrackers()) {
        auto state = trkr->state();
        if (best_state <= state) {
            valid = true;
            state = best_state;
            best_btrkr = trkr;
        }
    }
    return std::make_tuple(valid,best_btrkr);
}
void TrackerManager::deserialize_settings() {
    std::cout << "Reading settings from: " << settings_file << std::endl;
    TrackerManagerParameters params;
    if (file_exist(settings_file)) {
        std::ifstream infile(settings_file);
        std::string xmlData((std::istreambuf_iterator<char>(infile)),
                            std::istreambuf_iterator<char>());

        if (!xmls::Serializable::fromXML(xmlData, &params))
        {   // Deserialization not successful
            throw MyExit("Cannot read: " + settings_file);
        }
        TrackerManagerParameters tmp;
        auto v1 = params.getVersion();
        auto v2 = tmp.getVersion();
        if (v1 != v2) {
            throw MyExit("XML version difference detected from " + settings_file);
        }
        infile.close();
    } else {
        throw MyExit("File not found: " + settings_file);
    }

    max_points_per_frame = params.max_points_per_frame.value();
    roi_radius = params.roi_radius.value();
    motion_thresh = params.motion_thresh.value();
}

void TrackerManager::serialize_settings() {
    TrackerManagerParameters params;
    params.max_points_per_frame = max_points_per_frame;
    params.roi_radius = roi_radius;
    params.motion_thresh = motion_thresh;

    std::string xmlData = params.toXML();
    std::ofstream outfile = std::ofstream (settings_file);
    outfile << xmlData ;
    outfile.close();
}

void TrackerManager::read_false_positives() {
    string false_positive_rfn;
    if (replay_dir == "")
        false_positive_rfn = "./" + false_positive_fn;
    else
        false_positive_rfn = replay_dir + "/initial_" + false_positive_fn;

    if (file_exist(false_positive_rfn)) {

        std::string line = "";
        try {
            ifstream infile(false_positive_rfn);
            string heads;
            getline(infile, heads);

            while (getline(infile, line)) {
                auto splitted_line = split_csv_line(line);
                FalsePositive fp(static_cast<false_positive_type>(stoi(splitted_line.at(0))),stoi(splitted_line.at(2)),stof(splitted_line.at(3)),stof(splitted_line.at(4)),stof(splitted_line.at(5)) );
                false_positives.push_back(fp);
            }
            if (replay_dir == "")
                save_false_positives("./logging/initial_" + false_positive_fn);
            else
                save_false_positives("./logging/replay/initial_" + false_positive_fn);
        } catch (exception& exp ) {
            throw MyExit("Warning: could not read false positives file: " + false_positive_rfn + '\n' + "Line: " + string(exp.what()) + " at: " + line);
        }
    }
}
void TrackerManager::save_false_positives() {
    if (replay_dir == "") {
        save_false_positives("./logging/" + false_positive_fn);
        save_false_positives("../" + false_positive_fn);
    } else {
        save_false_positives("./logging/replay/" + false_positive_fn);
    }
}
void TrackerManager::save_false_positives(string false_positive_wfn) {
    std::ofstream fp_writer;
    fp_writer.open(false_positive_wfn,std::ofstream::out);
    fp_writer << "type;type_str;detection_count;imx;imy;im_size\n";
    for (auto fp : false_positives) {
        fp_writer << std::to_string(static_cast<int>(fp.type)) << ";" <<
                  false_positive_names[fp.type] << ";" <<
                  fp.detection_count  << ";" <<
                  fp.im_pt.x  << ";" <<
                  fp.im_pt.y  << ";" <<
                  fp.im_size  << "\n";
    }
    fp_writer.flush();
    fp_writer.close();
}

void TrackerManager::close () {
    if (initialized) {
        for (auto trkr : _trackers)
            trkr->close();
        save_false_positives();
        initialized = false;

    }
}

cv::Scalar TrackerManager::color_of_blob(ProcessedBlob blob) {
    if (blob.trackers.size() == 0 ) {
        if (blob.ignored)
            return cv::Scalar(0,128,0); // dark green
        else
            return cv::Scalar(255,255,55); // light blue
    } else if (blob.trackers.size()>1)
        return cv::Scalar(200,255,250);
    ItemTracker *trkr = blob.trackers.at(0);
    if (trkr->type() == tt_drone)
        return cv::Scalar(0,255,0); // green
    else if (trkr->type() == tt_insect)
        return cv::Scalar(0,0,255); // red
    else if (trkr->type() == tt_replay)
        return cv::Scalar(0,0,180); // dark red
    else if (trkr->type() == tt_virtualmoth)
        return cv::Scalar(255,0,255); // pink
    return cv::Scalar(0,0,0);
}

}
