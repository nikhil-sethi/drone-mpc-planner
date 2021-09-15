#include "trackermanager.h"
#include "interceptor.h"
using namespace cv;
using namespace std;

namespace tracking {

void TrackerManager::init(std::ofstream *logger,string replay_dir_,VisionData *visdat, Interceptor *iceptor) {
    _visdat = visdat;
    _logger = logger;

    _iceptor = iceptor;
    replay_dir = replay_dir_;

    if (pparams.has_screen || pparams.video_result) {
        _enable_viz_blob = false; // can be enable during runtime by key-pressing '['
        _enable_viz_tracker = false; // can be enable during runtime by key-pressing ']'
        enable_viz_motion = true; // part of the main visualisation
    }

    deserialize_settings();
    read_false_positives();

    _dtrkr = new DroneTracker();
    _dtrkr->init(_logger,_visdat,motion_thresh,1);
    _trackers.push_back(_dtrkr);

    (*_logger) << "trkrs_state;n_trackers;n_blobs;monsters;";
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
        find_blobs();
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
    _monster_alert = time_since_monsters > 0 && time - time_since_monsters < 30;
#ifdef PROLILING
    auto dur5 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - profile_t0).count();
#endif

    (*_logger) << static_cast<int16_t>(_mode) << ";" << _trackers.size() << ";" << _blobs.size() << ";" << time_since_monsters << ";";
#ifdef PROLILING
    (*_logger) << dur1 << ";"
               << dur2 << ";"
               << dur3 << ";"
               << dur4 << ";"
               << dur5 << ";";
#endif
    draw_trackers_viz();
    finish_vizs();
}

cv::Rect pre_select_roi(ImagePredictItem item, cv::Mat diff) {
    cv::Point max;
    cv::Point min;

    cv::Point2f pt = item.pt / pparams.imscalef;
    int roi_dist = item.size * 2;
    roi_dist = std::clamp(roi_dist, 10,300);

    int x_min = pt.x - roi_dist;
    x_min = std::clamp(x_min, 0, diff.cols);

    int x_max = pt.x + roi_dist;
    x_max = std::clamp(x_max, 0, diff.cols);

    int y_min = pt.y - roi_dist;
    y_min = std::clamp(y_min, 0, diff.rows);

    int y_max = pt.y + roi_dist;
    y_max =  std::clamp(y_max, 0, diff.rows);

    return cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);
}

//Pats Blob finder. Looks for a limited number of
//maxima in the motion image that are higher than a threshold, the area around the maximum
//is segmented from the background motion noise, and seen as a blob. In special cases it then
//tries if this blob can be further splitted if necessary.
void TrackerManager::find_blobs() {
    _blobs.clear();
    vizs_blobs.clear();
    cv::Mat diff = _visdat->diffL_small.clone();
    cv::Mat motion_filtered_noise_mapL = _visdat->motion_filtered_noise_mapL_small;

    auto insect_trackers = insecttrackers();
    if (_iceptor->target_insecttracker()) {
        auto it = std::find(insect_trackers.begin(), insect_trackers.end(), _iceptor->target_insecttracker());
        if (it != insect_trackers.end())
            std::iter_swap(insect_trackers.begin(),it);
    }

    auto roi_preselect_state = roi_state_drone;
    if (_mode == mode_locate_drone)
        roi_preselect_state = roi_state_blink;
    else if (_mode == mode_wait_for_insect ||_dtrkr->inactive()) {
        if (!insect_trackers.size())
            roi_preselect_state = roi_state_no_prior;
        else
            roi_preselect_state = roi_state_prior_insects;
    }

    double min_val_double, max_val_double;
    auto itrkr = insect_trackers.begin();
    unsigned int item_attempt = 0;
    int i = 0;
    while (i <= max_points_per_frame) {
        switch (roi_preselect_state) {
        case roi_state_drone: {
            item_attempt++;
            auto pre_select_roi_rect = pre_select_roi(_dtrkr->image_predict_item(),diff);
            Point min_pos_pre_roi,max_pos_pre_roi;
            minMaxLoc(diff(pre_select_roi_rect), &min_val_double, &max_val_double, &min_pos_pre_roi, &max_pos_pre_roi);
            uint8_t max_val = max_val_double;
            uint8_t motion_noise = motion_filtered_noise_mapL(pre_select_roi_rect).at<uint8_t>(max_pos_pre_roi.y,max_pos_pre_roi.x);
            bool max_is_valid = max_val > motion_noise+motion_thresh;
            if (max_is_valid) {
                cv::Point max_pos = max_pos_pre_roi + cv::Point(pre_select_roi_rect.x, pre_select_roi_rect.y);
                floodfind_and_remove(max_pos,max_val,motion_noise,diff,motion_filtered_noise_mapL);
            }
            if (!max_is_valid || item_attempt >= 3) {
                if (!insect_trackers.size())
                    roi_preselect_state = roi_state_no_prior;
                else
                    roi_preselect_state = roi_state_prior_insects;
            }
            i++;
            break;
        } case roi_state_prior_insects: {
            auto ipi = (*itrkr)->image_predict_item();
            auto pre_select_roi_rect = pre_select_roi(ipi,diff);
            Point min_pos_pre_roi,max_pos_pre_roi;
            minMaxLoc(diff(pre_select_roi_rect), &min_val_double, &max_val_double, &min_pos_pre_roi, &max_pos_pre_roi);
            int motion_noise = motion_filtered_noise_mapL(pre_select_roi_rect).at<uint8_t>(max_pos_pre_roi.y,max_pos_pre_roi.x);

            float chance = 1;
            if ((*itrkr)->tracking()) {
                chance += chance_multiplier_dist;
                if (ipi.valid && ipi.pixel_max < 1.5f * motion_noise)
                    chance += chance_multiplier_pixel_max;
            }
            if (static_cast<uint8_t>(max_val_double) > motion_noise+(motion_noise/chance)) {
                cv::Point max_pos = max_pos_pre_roi + cv::Point(pre_select_roi_rect.x, pre_select_roi_rect.y);
                floodfind_and_remove(max_pos,max_val_double,motion_noise,diff,motion_filtered_noise_mapL);
            }

            itrkr++;
            if (itrkr == insect_trackers.end())
                roi_preselect_state = roi_state_no_prior;
            i++;
            break;
        } case roi_state_blink: {
            Point min_pos,max_pos;
            minMaxLoc(diff, &min_val_double, &max_val_double, &min_pos, &max_pos);
            if (max_val_double > motion_thresh + dparams.drone_blink_strength) {
                motion_filtered_noise_mapL = motion_filtered_noise_mapL.clone();
                motion_filtered_noise_mapL = motion_thresh + dparams.drone_blink_strength;
                floodfind_and_remove(max_pos,max_val_double,0,diff,motion_filtered_noise_mapL);
            }
            i++;
            break;
        } case roi_state_no_prior: {
            Point min_pos,max_pos;
            minMaxLoc(diff, &min_val_double, &max_val_double, &min_pos, &max_pos);
            int motion_noise = motion_filtered_noise_mapL.at<uint8_t>(max_pos.y,max_pos.x);
            if (max_val_double > motion_noise * 2 + motion_thresh) {
                int max_noise = _visdat->max_noise(max_pos);
                if (max_val_double > max_noise + motion_thresh)
                    floodfind_and_remove(max_pos,max_val_double,motion_noise,diff,motion_filtered_noise_mapL);
                else  // probably noise spickle. Remove a very small area:
                    cv::circle(diff, max_pos, 3, Scalar(0), cv::FILLED);
            } else if (max_val_double > 2 * motion_thresh)
                cv::circle(diff, max_pos, 3, Scalar(0), cv::FILLED);
            else
                return;
            i++;
            break;
        }
        }

    }
}

std::tuple<float,bool,float> TrackerManager::tune_detection_radius(cv::Point maxt) {
    int roi_radius = default_roi_radius;
    bool enable_insect_drone_split = false;
    float drn_ins_split_thresh = 0;
    auto drn_predict = _dtrkr->image_predict_item();
    if (_mode == mode_locate_drone || (!drn_predict.valid && _mode == mode_hunt))
        return std::make_tuple(roi_radius,enable_insect_drone_split,drn_ins_split_thresh); // take a roi suited to prioritize finding (back) the drone

    cv::Point2f max_unscaled = maxt*pparams.imscalef;
    ItemTracker * likely_trkr = NULL;
    float best_im_dist = INFINITY;
    if (!drn_predict.valid || best_im_dist > drn_predict.size *0.45f ) { //0.45 -> divide by 2 and 10% margin
        for (auto trkr : _trackers) {
            if (trkr->image_predict_item().valid && trkr->type() != tt_replay && trkr->type() != tt_virtualmoth) {
                float im_dist = normf(trkr->image_predict_item().pt - max_unscaled);
                if (im_dist < best_im_dist) {
                    best_im_dist = im_dist;
                    likely_trkr = trkr;
                }
            }
        }
    }
    if (!likely_trkr || (best_im_dist > 2*drn_predict.size && drn_predict.valid)) {
        roi_radius = default_roi_radius/2; // we expect a (new) insect here, default_roi_radius is tuned for the drone size. So set it a bit smaller.
    } else if (likely_trkr->type() != tt_drone) {
        roi_radius = ceilf(likely_trkr->image_predict_item().size * 0.8f) / pparams.imscalef;
        roi_radius = std::clamp(roi_radius,5,100);
        if (drn_predict.valid) {
            auto target_trkr = _iceptor->target_insecttracker();
            if (target_trkr) {
                if (target_trkr->uid() == likely_trkr->uid() && target_trkr->type() == tt_insect) {
                    auto image_predict_item = target_trkr->image_predict_item();
                    if (normf(drn_predict.pt - image_predict_item.pt) < roi_radius ) {
                        enable_insect_drone_split = true;
                        drn_ins_split_thresh = image_predict_item.pixel_max*0.2f;
                    }
                }
            }
        }
    } else {
        roi_radius = ceilf(drn_predict.size * 0.8f) / pparams.imscalef;
        roi_radius = std::clamp(roi_radius,default_roi_radius/2,100);
    }
    return std::make_tuple(roi_radius,enable_insect_drone_split,drn_ins_split_thresh);
}

void TrackerManager::floodfind_and_remove(cv::Point seed_max_pos, uint8_t seed_max_val, uint8_t motion_noise,cv::Mat diff, cv::Mat motion_filtered_noise_mapL) {
    cv::Point2i bound_min = seed_max_pos;
    cv::Point2i bound_max = seed_max_pos;

    std::vector<cv::Point2i> todo_pts;

    int npixels = 0;
    uint32_t motion_sum = 0;
    cv::Point2l COG = {0};

    Point min_pos,max_pos = seed_max_pos;
    double min_val;
    uint8_t max_val = seed_max_val;

    while (max_val > motion_filtered_noise_mapL.at<uint8_t>(max_pos)) {
        todo_pts.push_back(max_pos);
        while (!todo_pts.empty()) {
            auto p = todo_pts.back();
            todo_pts.pop_back();
            if (diff.at<uint8_t>(p) > motion_filtered_noise_mapL.at<uint8_t>(p)) {
                npixels++;
                motion_sum += diff.at<uint8_t>(p) - motion_filtered_noise_mapL.at<uint8_t>(p);
                COG.x+=p.x;
                COG.y+=p.y;
                diff.at<uint8_t>(p) = 0;
                if (p.x < bound_min.x)
                    bound_min.x = p.x;
                else if (p.x > bound_max.x)
                    bound_max.x = p.x;
                if (p.y < bound_min.y)
                    bound_min.y = p.y;
                else if (p.y > bound_max.y)
                    bound_max.y = p.y;

                //add pixel directly above, below or besides the max point
                if (p.x > 0)
                    todo_pts.push_back(cv::Point2i(p.x-1,p.y));
                if (p.x < diff.cols-1)
                    todo_pts.push_back(cv::Point2i(p.x+1,p.y));
                if (p.y > 0)
                    todo_pts.push_back(cv::Point2i(p.x,p.y-1));
                if (p.y < diff.rows-1)
                    todo_pts.push_back(cv::Point2i(p.x,p.y+1));

                // Add pixel to the diagnonal to the max point, jump one pixel for efficiency reasons and better blob merge behavior
                // (We've now adopted to look for additional maxima in the bounding box, but this may still proove usefull if we go look for a drne/insect splitted blob)
                // if (p.x > 2 && p.y > 2)
                //     todo_pts.push_back(cv::Point2i(p.x-3,p.y-3));
                // if (p.x < diff.cols-3 && p.y < diff.rows-3)
                //     todo_pts.push_back(cv::Point2i(p.x+3,p.y+3));
                // if (p.x < diff.cols-3 && p.y > 2)
                //     todo_pts.push_back(cv::Point2i(p.x+3,p.y-3));
                // if (p.x > 2 && p.y < diff.rows-3)
                //     todo_pts.push_back(cv::Point2i(p.x-3,p.y+3));
            }
        }
        cv::Rect bounding_box = clamp_rect(cv::Rect(bound_min + cv::Point(-4,-4),bound_max + cv::Point(4,4)),diff.cols,diff.rows);
        double max_val_double;
        cv::minMaxLoc(diff(bounding_box), &min_val, &max_val_double, &min_pos, &max_pos);
        max_val = max_val_double;
        max_pos = max_pos + cv::Point(bounding_box.x,bounding_box.y);
    }
    if (!npixels)
        COG = seed_max_pos;
    else
        COG /= npixels;
    float size = normf(bound_max - bound_min);
    _blobs.push_back(tracking::BlobProps(COG,size,npixels,motion_sum,seed_max_val,motion_noise,_visdat->overexposed(COG),_visdat->frame_id));

    if (_enable_viz_blob) {
        bound_max+=Point(8,8);
        bound_min-=Point(8,8);
        cv::Rect bounding_box = clamp_rect(cv::Rect(bound_min,bound_max),diff.cols,diff.rows);

        Mat viz;
        Rect bounding_box_unscaled = clamp_rect(cv::Rect(bound_min*pparams.imscalef,bound_max*pparams.imscalef),IMG_W,IMG_H);;
        cv::Mat frameL_roi = _visdat->frameL(bounding_box_unscaled);
        cv::Mat frameL_small_roi;
        cv::resize(frameL_roi,frameL_small_roi,cv::Size(frameL_roi.cols/pparams.imscalef,frameL_roi.rows/pparams.imscalef));

        cv::Mat overexposed_roi;
        if (_visdat->overexposed_mapL.cols) {
            cv::Mat tmp = _visdat->overexposed_mapL(bounding_box_unscaled);
            cv::resize(tmp,overexposed_roi,cv::Size(frameL_roi.cols/pparams.imscalef,frameL_roi.rows/pparams.imscalef));
        } else
            overexposed_roi = cv::Mat::zeros(bounding_box.size(),CV_8UC1);

        cv::Mat diff_annotated = diff(bounding_box);
        cvtColor(diff_annotated,diff_annotated,cv::COLOR_GRAY2BGR);
        cv::insertChannel(_visdat->diffL_small(bounding_box),diff_annotated,2);
        diff_annotated*=10;

        viz = create_row_image({diff_annotated,_visdat->diffL_small(bounding_box)*10,frameL_small_roi,motion_filtered_noise_mapL(bounding_box)*10,overexposed_roi},CV_8UC3,viz_blobs_resizef);
        cv::Point2i COG_viz = cv::Point(COG.x-bounding_box.x,COG.y -bounding_box.y) * viz_blobs_resizef;
        cv::circle(viz,COG_viz,1,Scalar(0,255,0),2,cv::FILLED);
        vizs_blobs.push_back(viz);
    }
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
        if (_mode == mode_locate_drone) {
            rematch_blink_tracker(&pbs,time);
            create_new_blink_trackers(&pbs,time);
        } else {
            rematch_drone_tracker(&pbs,drone_is_active,time);
            flag_used_static_ignores(&pbs);
            create_new_insect_trackers(&pbs,time);
        }
    }

    draw_motion_viz(&pbs,time);
    draw_blob_viz(&pbs);

    for (auto trkr : _trackers)
        trkr->invalidize(false);
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
                    if (_enable_viz_blob && trkr->viz_id() < 6) {
                        cv::Mat viz = vizs_blobs.at(blob.id);
                        cv::Point2i pt(viz.cols/5 + 2,viz.rows - 20*(trkr->viz_id()+1));
                        putText(viz,"#" + std::to_string(trkr->viz_id()) + ": " + to_string_with_precision(score,2),pt,FONT_HERSHEY_SIMPLEX,0.4,tracker_color(trkr));
                    }
                } else if (_enable_viz_blob) {
                    cv::Mat viz = vizs_blobs.at(blob.id);
                    cv::Point2i pt(viz.cols/5+2,viz.rows -6-14*(trkr->viz_id()+1));
                    putText(viz,"Ign.",pt,FONT_HERSHEY_SIMPLEX,0.3,tracker_color(trkr));
                }
            }
            if (best_trkr_score < trkr->score_threshold() ) {
                best_blob->trackers.push_back(trkr);
                trkr->calc_world_item(best_blob->props,time);
                tracking::ImageItem image_item(*(best_blob->props),_visdat->frame_id,best_trkr_score,best_blob->id);
                tracking::WorldItem w(image_item,best_blob->props->world_props);
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
                                float dist = cv::norm(blob_i.pt()-blob_j.pt()) * pparams.imscalef;
                                if (dist < _dtrkr->image_predict_item().size) {
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
                    if (trkr->uid() != best_tracker->uid())
                        trkr->invalidize(true);
                }
                blob_i.trackers.clear();
                blob_i.trackers.push_back(best_tracker);
            }
        }
    }

}
void TrackerManager::rematch_drone_tracker(std::vector<ProcessedBlob> *pbs,bool drone_is_active, double time) {
    auto trkr = _dtrkr;
    if (!_dtrkr->tracking() && tracker_active(_dtrkr, drone_is_active)) {
        for (auto &blob : *pbs) {
            if (!blob.tracked() && (!blob.props->false_positive)) {

                //check against static ignore points
                auto props = blob.props;
                bool in_im_ignore_zone = _dtrkr->check_ignore_blobs(props);
                if (in_im_ignore_zone)
                    blob.ignored = true;

                if (!in_im_ignore_zone) {

                    auto score = trkr->score(props);
                    if (score<trkr->score_threshold()) {

                        trkr->calc_world_item(props,time);
                        tracking::WorldItem w(tracking::ImageItem(*props,_visdat->frame_id,0,blob.id),props->world_props);
                        if (w.valid) {
                            trkr->world_item(w);
                            blob.trackers.push_back(trkr);

                            // There may be other interesting blobs to consider but since there is no
                            // history available at this point, its hard to calculate which one would
                            // be better. So just pick the first one...:
                            break;
                        }
                    }
                }

            }
        }
    }
}
void TrackerManager::rematch_blink_tracker(std::vector<ProcessedBlob> *pbs, double time) {
    for (auto trkr : _trackers) {
        if (trkr->type() == tt_blink) {
            for (auto &blob : *pbs) {
                if (!blob.tracked()) {

                    //check against static ignore points
                    auto props = blob.props;
                    bool in_im_ignore_zone = trkr->check_ignore_blobs(props);
                    if (in_im_ignore_zone)
                        blob.ignored = true;

                    if (!in_im_ignore_zone) {
                        auto score = trkr->score(props);
                        if (score<trkr->score_threshold()) {
                            trkr->calc_world_item(props,time);
                            tracking::WorldItem w(tracking::ImageItem(*props,_visdat->frame_id,0,blob.id),props->world_props);
                            if (w.valid) {
                                trkr->world_item(w);
                                blob.trackers.push_back(trkr);
                                break;
                            }
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
    if (_mode != mode_drone_only && _visdat->no_recent_large_brightness_events(time)) {
        for (auto &blob : *pbs) {
            auto props = blob.props;
            if (!blob.tracked() && (!props->in_overexposed_area || _mode == mode_locate_drone) && (!props->false_positive || _mode == mode_locate_drone) && !blob.ignored && _trackers.size() < 30) { // if so, start tracking it!
                float im_dist_to_drone;
                if (_dtrkr->tracking()) {
                    im_dist_to_drone = normf(_dtrkr->image_item().pt()-props->pt_unscaled());
                } else {
                    if (_dtrkr->pad_location_valid())
                        im_dist_to_drone = normf(_dtrkr->pad_im_location() - props->pt_unscaled());
                    else
                        im_dist_to_drone = INFINITY;
                }
                if (im_dist_to_drone > InsectTracker::new_tracker_drone_ignore_zone_size_im && blob.pixel_max() > blob.motion_noise()+ motion_thresh ) {
                    InsectTracker *it;
                    it = new InsectTracker();
                    it->init(next_insecttrkr_id,_visdat,motion_thresh,_trackers.size(),_enable_draw_stereo_viz);
                    it->calc_world_item(props,time);
                    if (!props->world_props.bkg_check_ok)
                        tracking::WorldItem w(tracking::ImageItem(*props,_visdat->frame_id,-1,blob.id),props->world_props);

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
                            world_dist_to_drone = normf(_dtrkr->pad_location() - props->world_props.pt());

                        if (world_dist_to_drone > InsectTracker::new_tracker_drone_ignore_zone_size_world && im_dist_to_drone > InsectTracker::new_tracker_drone_ignore_zone_size_im) {
                            it->init_logger();
                            std::cout << "Keeping insecttracker: " << next_insecttrkr_id << " uid: " << it->uid() << std::endl;
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
void TrackerManager::create_new_blink_trackers(std::vector<ProcessedBlob> *pbs, double time) {
    //see if there are still blobs left untracked, create new trackers for them
    for (auto &blob : *pbs) {
        auto props = blob.props;
        if (!blob.tracked() && (!props->in_overexposed_area || _mode == mode_locate_drone) && (!props->false_positive || _mode == mode_locate_drone) && !blob.ignored && _trackers.size() < 30) { // if so, start tracking it!
            bool too_close = false;
            for (auto btrkr : _trackers) {
                if (btrkr->type() == tt_blink) {
                    if (normf(btrkr->image_item().pt() - props->pt_unscaled())  < default_roi_radius * pparams.imscalef) {
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
                if (fpt == fp_static_location && itrkr->track().size() && !_monster_alert) {
                    _fp_statics_count++;
                    false_positives.push_back(FalsePositive(itrkr->track().begin()->world_item,fpt,time));
                } else if (fpt == fp_short_detection && !_monster_alert) {
                    _fp_shorts_count++;
                } else if (fpt == fp_too_big || fpt == fp_too_far) {
                    _fp_monsters_count++;
                } else if (!_monster_alert) {
                    _insects_count++;
                }
            }
            trkr->close();
            _trackers.erase(_trackers.begin() + i);
            std::cout << "Deleting tracker: " << trkr->uid() << std::endl;
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
            if (itrkr->monster_alert())
                time_since_monsters = time;
        } else if(_trackers.at(i)->type() == tt_replay) {
            ReplayTracker *rtrkr = static_cast<ReplayTracker *>(_trackers.at(i));
            rtrkr->update(time);
        } else if(_trackers.at(i)->type() == tt_virtualmoth) {
            VirtualMothTracker*vtrkr = static_cast<VirtualMothTracker*>(_trackers.at(i));
            vtrkr->update(time);
        } else if (_trackers.at(i)->type() == tt_blink) {
            BlinkTracker *btrkr = static_cast<BlinkTracker *>(_trackers.at(i));
            btrkr->update(time);
            if (_mode == mode_locate_drone) {
                if (btrkr->state() == BlinkTracker::bds_found) {
                    _dtrkr->set_pad_location_from_blink(btrkr->world_item().pt);
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
    if (enable_viz_motion)
        cv::cvtColor(_visdat->diffL*10,diff_viz,cv::COLOR_GRAY2BGR);
    reset_trkr_viz_ids();
}
void TrackerManager::draw_trackers_viz() {
    if (_enable_viz_tracker) {
        int h = 0;
        cv::Scalar white = cv::Scalar(255,255,255);
        cv::Scalar red = cv::Scalar(0,0,255);
        int size_text = 12;
        int descr_w = 320;
        int descr_h = 10*size_text; // -> lines of text in description frame
        int req_h=0, req_w=descr_w;
        for (auto trkr : _trackers) { //dry run to figure out size of the final viz
            auto image_item = trkr->image_item();
            if (image_item.valid) {
                auto [roiL,roiR,resize_factor] = calc_trkr_viz_roi(image_item);
                int height = roiL.height;
                if (roiR.height> height)
                    height = roiR.height;
                int width = roiL.width;
                if (roiR.width> width)
                    width = roiR.width;
                if (height*2*resize_factor>descr_h)
                    req_h+=height*2*resize_factor;
                else
                    req_h+=descr_h;
                if (width*4*resize_factor+descr_w > req_w)
                    req_w = width*4*resize_factor+descr_w;
            } else
                req_h+=descr_h;
        }

        cv::Mat viz_tracker = cv::Mat::zeros(cv::Size(req_w,req_h),CV_8UC3);
        for (auto trkr : _trackers) {
            auto image_item = trkr->image_item();
            auto wti = trkr->world_item();
            cv::Rect roiD(0,h,descr_w,descr_h);
            cv::Mat viz_descr =  viz_tracker(roiD);
            int y_text = 1;
            putText(viz_descr,"Tracker " + std::to_string(trkr->viz_id()) + ": " + trkr->name(),cv::Point2i(1,y_text++ * size_text),FONT_HERSHEY_DUPLEX,0.4,white);

            if (image_item.valid) {
                auto [roiL,roiR,resize_factor] = calc_trkr_viz_roi(image_item);

                std::vector<cv::Mat> vizsL;
                vizsL.push_back(_visdat->frameL(roiL));
                vizsL.push_back(_visdat->diffL(roiL)*10);
                if (_visdat->motion_filtered_noise_mapL.cols)
                    vizsL.push_back(_visdat->motion_filtered_noise_mapL(roiL)*10);
                if (_visdat->overexposed_mapL.cols)
                    vizsL.push_back(_visdat->overexposed_mapL(roiL));
                cv::Mat vizL = create_row_image(vizsL,CV_8UC3,1);

                std::vector<cv::Mat> vizsR;
                vizsR.push_back(_visdat->frameR(roiR));
                vizsR.push_back(_visdat->diffR(roiR)*10);
                if (_visdat->motion_filtered_noise_mapR.cols)
                    vizsR.push_back(_visdat->motion_filtered_noise_mapR(roiR)*10);
                if (_visdat->overexposed_mapR.cols)
                    vizsR.push_back(_visdat->overexposed_mapR(roiR));
                cv::Mat vizR = create_row_image(vizsR,CV_8UC3,1);

                cv::Mat vizLR = create_column_image({vizL,vizR},CV_8UC3,resize_factor);

                cv::Rect roiLR(roiD.width,h,vizLR.cols,vizLR.rows);
                vizLR.copyTo(viz_tracker(roiLR));
                if (vizLR.rows>roiD.height)
                    h+=vizLR.rows;
                else
                    h+=roiD.height;

                putText(viz_descr,"blob id -> tr_uid: " + std::to_string(image_item.blob_id) + " -> " + std::to_string(trkr->uid()),cv::Point2i(1,y_text++ * size_text),FONT_HERSHEY_SIMPLEX,0.4,white);
                putText(viz_descr,std::to_string(resize_factor) + "x",cv::Point2i(viz_descr.cols-16,size_text),FONT_HERSHEY_SIMPLEX,0.4,red);
                putText(viz_descr,"Score: " + to_string_with_precision(image_item.score,2),cv::Point2i(1,y_text++ * size_text),FONT_HERSHEY_SIMPLEX,0.4,white);
                putText(viz_descr,"# frames: " + std::to_string(trkr->n_frames_tracking()) + " / " + std::to_string(trkr->n_frames()),cv::Point2i(1,y_text++ * size_text),FONT_HERSHEY_SIMPLEX,0.4,white);
                putText(viz_descr,"Size: " + to_string_with_precision(image_item.size,1),cv::Point2i(1,y_text++ * size_text),FONT_HERSHEY_SIMPLEX,0.4,white);
                putText(viz_descr,"Max: " + to_string_with_precision(image_item.pixel_max,1),cv::Point2i(1,y_text++ * size_text),FONT_HERSHEY_SIMPLEX,0.4,white);
                putText(viz_descr,"i: [" + to_string_with_precision(image_item.x,1) + ", " + to_string_with_precision(image_item.y,1) + "], " +  to_string_with_precision(image_item.disparity,2),cv::Point2i(1,y_text++ * size_text),FONT_HERSHEY_SIMPLEX,0.4,white);
                putText(viz_descr,"w: [" + to_string_with_precision(wti.pt.x,2) + ", " + to_string_with_precision(wti.pt.y,2) + ", " + to_string_with_precision(wti.pt.z,2) + "] ||" + to_string_with_precision(wti.radius,3) + "||",cv::Point2i(1,y_text++ * size_text),FONT_HERSHEY_SIMPLEX,0.4,white);

                int text_w = 0;
                if (trkr->image_predict_item().out_of_image) {
                    std::string flag = "OUT OF IMAGE";
                    putText(viz_descr,flag,cv::Point2i(1,y_text++ * size_text),FONT_HERSHEY_TRIPLEX,0.4,cv::Scalar(0,0,255));
                    int baseline;
                    Size text_size = getTextSize(flag,FONT_HERSHEY_TRIPLEX,0.75,2,&baseline);
                    text_w+=text_size.width;
                } else if (trkr->image_predict_item().out_of_image_right) {
                    std::string flag = "OUT OF RIGHT IMAGE";
                    putText(viz_descr,flag,cv::Point2i(1,y_text++ * size_text),FONT_HERSHEY_TRIPLEX,0.4,cv::Scalar(0,0,255));
                    int baseline;
                    Size text_size = getTextSize(flag,FONT_HERSHEY_TRIPLEX,0.75,2,&baseline);
                    text_w+=text_size.width;
                }
                if (trkr->type() == tt_insect) {
                    InsectTracker *itrkr = static_cast<InsectTracker *> (trkr);
                    if (itrkr->false_positive()) {
                        std::string flag = "";
                        if (text_w > 0)
                            flag+= " | ";
                        flag+=false_positive_names[itrkr->false_positive()];
                        putText(viz_descr,flag,cv::Point2i(1,y_text++ * size_text),FONT_HERSHEY_TRIPLEX,0.4,cv::Scalar(255,0,0));
                        int baseline;
                        Size text_size = getTextSize(flag,FONT_HERSHEY_TRIPLEX,0.75,2,&baseline);
                        text_w+=text_size.width;
                    }
                }
                if (image_item.blob_is_fused) {
                    std::string flag = "";
                    if (text_w > 0)
                        flag+= " | ";
                    flag+="Fused";
                    putText(viz_descr,flag,cv::Point2i(1,y_text++ * size_text),FONT_HERSHEY_TRIPLEX,0.4,cv::Scalar(0,128,255));
                    int baseline;
                    Size text_size = getTextSize(flag,FONT_HERSHEY_TRIPLEX,0.75,2,&baseline);
                    text_w+=text_size.width;
                }

            } else {
                putText(viz_descr,"No blob found",cv::Point2i(1,y_text++ * size_text),FONT_HERSHEY_SIMPLEX,0.4,red);
                putText(viz_descr,"tr_uid: " + std::to_string(trkr->uid()),cv::Point2i(1,y_text++ * size_text),FONT_HERSHEY_SIMPLEX,0.4,white);
                putText(viz_descr,"Lost: " + std::to_string(trkr->n_frames_lost()) + " / " + std::to_string(trkr->n_frames()),cv::Point2i(1,y_text++ * size_text),FONT_HERSHEY_SIMPLEX,0.4,white);
                int text_w = 0;
                if (trkr->image_predict_item().out_of_image) {
                    std::string flag = "OUT OF IMAGE";
                    putText(viz_descr,flag,cv::Point2i(1,y_text++ * size_text),FONT_HERSHEY_TRIPLEX,0.4,cv::Scalar(0,0,255));
                    int baseline;
                    Size text_size = getTextSize(flag,FONT_HERSHEY_TRIPLEX,0.75,2,&baseline);
                    text_w+=text_size.width;
                }
                if (trkr->type() == tt_insect) {
                    InsectTracker *itrkr = static_cast<InsectTracker *> (trkr);
                    if (itrkr->false_positive()) {
                        std::string flag = "";
                        if (text_w > 0)
                            flag+= " | ";
                        flag+=false_positive_names[itrkr->false_positive()];
                        putText(viz_descr,flag,cv::Point2i(1,y_text++ * size_text),FONT_HERSHEY_TRIPLEX,0.4,cv::Scalar(255,0,0));
                        int baseline;
                        Size text_size = getTextSize(flag,FONT_HERSHEY_TRIPLEX,0.75,2,&baseline);
                        text_w+=text_size.width;
                    }
                }
                h+=roiD.height;
            }

        }
        viz_trkrs_buf = viz_tracker.clone();
    }
}
void TrackerManager::draw_motion_viz(std::vector<ProcessedBlob> *pbs, double time) {
    if (enable_viz_motion) {
        for (auto blob : *pbs) {
            std::string s = std::to_string(blob.id);
            if (blob.trackers.size()>0) {
                s = s + "->";
                for (auto trkr : blob.trackers) {
                    s = s + std::to_string(trkr->uid()) + "+";
                }
                s.pop_back();
            }
            cv::Point target_viz_p = (blob.props->pt_unscaled() + cv::Point2f(10,-10));
            putText(diff_viz,blob.prefix() +  s,target_viz_p,FONT_HERSHEY_SIMPLEX,0.4,blob.color(),2);
            cv::line(diff_viz,target_viz_p,blob.props->pt_unscaled(),blob.color());

        }
        for (auto trkr : replaytrackers()) {
            cv::Point target_viz_p = (trkr->image_item().pt() + cv::Point2f(10,-10));
            putText(diff_viz," r",target_viz_p,FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(0,0,180),2);
            cv::line(diff_viz,target_viz_p,trkr->image_item().pt(),cv::Scalar(0,0,180));
        }
        for (auto trkr : virtualmothtrackers()) {
            cv::Point target_viz_p = (trkr->image_item().pt() + cv::Point2f(10,-10));
            putText(diff_viz," v",target_viz_p,FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(0,0,180),2);
            cv::line(diff_viz,target_viz_p,trkr->image_item().pt(),cv::Scalar(0,0,180));
        }

        auto blue =cv::Scalar(255,0,0);
        for (auto fp : false_positives) {
            auto p_center = fp.im_pt;
            cv::circle(diff_viz,p_center,fp.im_size,blue);
            cv::putText(diff_viz,to_string_with_precision(time-fp.last_seen_time,1) + ", #" + std::to_string(fp.detection_count),p_center+cv::Point2f(10,0),FONT_HERSHEY_SIMPLEX,0.4,blue,2);
        }
    }
}
void TrackerManager::draw_blob_viz(std::vector<ProcessedBlob> *pbs) {
    if (_enable_viz_blob) {
        for (auto blob : *pbs) {
            auto wblob = blob.props->world_props;
            std::string msg_str = std::to_string(blob.id);
            cv::Mat viz = vizs_blobs.at(blob.id);
            if (!wblob.valid && !blob.ignored) {
                if (blob.props->in_overexposed_area && !blob.tracked()) {
                    msg_str = msg_str + "exp.";
                } else  if (!wblob.im_pos_ok && !blob.tracked())
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
                putText(viz,msg_str,cv::Point2i(viz.cols/5+2,viz.rows - 8),FONT_HERSHEY_SIMPLEX,0.4,blob.color());
            }
            if (wblob.valid) {
                std::string coor_str = "[" + to_string_with_precision(wblob.x,2) + ", " + to_string_with_precision(wblob.y,2) + ", " + to_string_with_precision(wblob.z,2) +"]";
                putText(viz,coor_str,cv::Point2i(viz.cols/5+2,viz.rows - 8),FONT_HERSHEY_SIMPLEX,0.4,blob.color());
            }
        }
    }
}
std::tuple<Rect,Rect,int> TrackerManager::calc_trkr_viz_roi(ImageItem image_item) {
    cv::Rect roiL(image_item.x - image_item.size-5,image_item.y - image_item.size-5,image_item.size*2+10,image_item.size*2+10);
    roiL = clamp_rect(roiL,IMG_W,IMG_H);
    cv::Rect roiR(image_item.x - image_item.disparity - image_item.size-5,image_item.y - image_item.size-5,image_item.size*2+10,image_item.size*2+10);
    roiR = clamp_rect(roiR,IMG_W,IMG_H);
    int resize_factor =1 ;
    if (roiL.height < 15)
        resize_factor = 4;
    else if (roiL.height < 30)
        resize_factor = 2;

    return std::make_tuple(roiL,roiR,resize_factor);
}
void TrackerManager::finish_vizs() {
    if (_enable_viz_blob && vizs_blobs.size()>0)
        viz_max_points = create_column_image(vizs_blobs,CV_8UC3,1);
    else if(_enable_viz_blob)
        viz_max_points = cv::Mat::zeros(5,100,CV_8UC3);
    if (enable_viz_motion)
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
std::vector<VirtualMothTracker *> TrackerManager::virtualmothtrackers() {
    std::vector<VirtualMothTracker *> res;
    for (auto trkr : _trackers) {
        if (trkr->type() == tt_virtualmoth) {
            res.push_back(static_cast<VirtualMothTracker *>(trkr));
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
    motion_thresh = params.motion_thresh.value();
}

void TrackerManager::serialize_settings() {
    TrackerManagerParameters params;
    params.max_points_per_frame = max_points_per_frame;
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
