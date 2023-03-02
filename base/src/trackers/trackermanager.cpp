#include "trackermanager.h"
#include "interceptor.h"
using namespace cv;
using namespace std;

namespace tracking {

void TrackerManager::init(std::ofstream *logger, string replay_dir_, VisionData *visdat, Interceptor *interceptor) {
    _visdat = visdat;
    _logger = logger;

    _iceptor = interceptor;
    replay_dir = replay_dir_;

    if (pparams.has_screen || pparams.video_render) {
        _enable_viz_blob = false; // can be enable during runtime by key-pressing '['
        _enable_viz_tracker = false; // can be enable during runtime by key-pressing ']'
        enable_viz_motion = true; // part of the main visualisation
    }

    deserialize_settings();
    read_false_positives();

    (*_logger) << "trackers_state_str;trackers_state;n_trackers;n_blobs;monsters;";
    initialized = true;
}

void TrackerManager::start_drone_tracking(DroneTracker *dtrk) {
    dtrk->init(_visdat, _motion_thresh, 1);
    _trackers.push_back(dtrk);
}
void TrackerManager::stop_drone_tracking(DroneTracker *dtrk) {
    _trackers.erase(std::remove_if(_trackers.begin(), _trackers.end(), [&](ItemTracker * trkr) {return (trkr->uid() == dtrk->uid());}));
}

void TrackerManager::update(double time) {
#ifdef PATS_PROFILING
    std::chrono::_V2::system_clock::time_point t_start_trackers = std::chrono::high_resolution_clock::now();
#endif
    prep_vizs();
    if (_mode != t_idle) {
        find_blobs();
        if (_visdat->first_frame()) {
            for (auto  blob : _blobs)
                _visdat->add_start_reset_spot_on_motion_map(blob.pt_unscaled(), blob.size_unscaled() / 2.f + 2);
        }
        collect_static_ignores();
        erase_dissipated_fps(time);
        match_blobs_to_trackers(time);
    }
    update_trackers(time, _visdat->frame_id);
    _monster_alert = time_since_monsters > 0 && time - time_since_monsters < 30;
    (*_logger) << trackermanager_mode_names[_mode] << ";" << static_cast<uint16_t>(_mode) << ";" << _trackers.size() << ";" << _blobs.size() << ";" << time_since_monsters << ";";
    draw_trackers_viz();
    finish_vizs();
#ifdef PATS_PROFILING
    std::chrono::_V2::system_clock::time_point t_end_trackers = std::chrono::high_resolution_clock::now();
    std::cout << "timing (update_trackers): " << (t_end_trackers - t_start_trackers).count() * 1e-6 << "ms" << std::endl;
#endif
}

cv::Rect pre_select_roi(ImagePredictItem item, cv::Mat diff) {
    cv::Point max;
    cv::Point min;

    cv::Point2f pt = item.pt / im_scaler;
    int roi_dist = item.size * 2;
    roi_dist = std::clamp(roi_dist, 10, 300);

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
    auto drone_trackers = dronetrackers();
    if (_iceptor->target_insecttracker()) {
        auto it = std::find(insect_trackers.begin(), insect_trackers.end(), _iceptor->target_insecttracker());
        if (it != insect_trackers.end())
            std::iter_swap(insect_trackers.begin(), it);
    }

    auto roi_preselect_state = roi_state_drone;
    if (_mode == t_locate_drone)
        roi_preselect_state = roi_state_blink;
    else if (drone_trackers.size())
        roi_preselect_state = roi_state_drone;
    else if (!insect_trackers.size())
        roi_preselect_state = roi_state_no_prior;
    else
        roi_preselect_state = roi_state_prior_insects;


    double min_val_double, max_val_double;
    auto itrkr = insect_trackers.begin();
    auto dtrkr = drone_trackers.begin();
    unsigned int item_attempt = 0;
    int i = 0;
    while (i <= max_points_per_frame) {
        switch (roi_preselect_state) {
            case roi_state_drone: {
                    item_attempt++;
                    auto pre_select_roi_rect = pre_select_roi((*dtrkr)->image_predict_item(), diff);
                    Point min_pos_pre_roi, max_pos_pre_roi;
                    minMaxLoc(diff(pre_select_roi_rect), &min_val_double, &max_val_double, &min_pos_pre_roi, &max_pos_pre_roi);
                    uint8_t max_val = max_val_double;
                    uint8_t motion_noise = motion_filtered_noise_mapL(pre_select_roi_rect).at<uint8_t>(max_pos_pre_roi.y, max_pos_pre_roi.x);
                    bool max_is_valid = max_val > motion_noise + _motion_thresh;
                    if (max_is_valid) {
                        cv::Point max_pos = max_pos_pre_roi + cv::Point(pre_select_roi_rect.x, pre_select_roi_rect.y);
                        floodfind_and_remove(max_pos, max_val, motion_noise, diff, motion_filtered_noise_mapL);
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
                    auto pre_select_roi_rect = pre_select_roi(ipi, diff);
                    Point min_pos_pre_roi, max_pos_pre_roi;
                    minMaxLoc(diff(pre_select_roi_rect), &min_val_double, &max_val_double, &min_pos_pre_roi, &max_pos_pre_roi);
                    uint8_t max_val = max_val_double;
                    int motion_noise = motion_filtered_noise_mapL(pre_select_roi_rect).at<uint8_t>(max_pos_pre_roi.y, max_pos_pre_roi.x);

                    float chance = 1;
                    if ((*itrkr)->tracking()) {
                        chance += chance_multiplier_dist;
                        if (ipi.valid && ipi.pixel_max < 1.5f * motion_noise)
                            chance += chance_multiplier_pixel_max;
                    }
                    if (max_val > motion_noise + (motion_noise / chance)) {
                        cv::Point max_pos = max_pos_pre_roi + cv::Point(pre_select_roi_rect.x, pre_select_roi_rect.y);
                        floodfind_and_remove(max_pos, max_val, motion_noise, diff, motion_filtered_noise_mapL);
                    }

                    itrkr++;
                    if (itrkr == insect_trackers.end())
                        roi_preselect_state = roi_state_no_prior;
                    i++;
                    break;
            } case roi_state_blink: {
                    Point min_pos, max_pos;
                    minMaxLoc(diff, &min_val_double, &max_val_double, &min_pos, &max_pos);
                    if (max_val_double > _motion_thresh + dparams.drone_blink_strength) {
                        motion_filtered_noise_mapL = motion_filtered_noise_mapL.clone();
                        motion_filtered_noise_mapL = _motion_thresh + dparams.drone_blink_strength;
                        floodfind_and_remove(max_pos, max_val_double, 0, diff, motion_filtered_noise_mapL);
                    }
                    i++;
                    break;
            } case roi_state_no_prior: {
                    Point min_pos, max_pos;
                    minMaxLoc(diff, &min_val_double, &max_val_double, &min_pos, &max_pos);
                    int motion_noise = motion_filtered_noise_mapL.at<uint8_t>(max_pos.y, max_pos.x);
                    if (max_val_double > motion_noise * 2 + _motion_thresh) {
                        int max_noise = _visdat->max_noise(max_pos);
                        if (max_val_double > max_noise + _motion_thresh)
                            floodfind_and_remove(max_pos, max_val_double, motion_noise, diff, motion_filtered_noise_mapL);
                        else  // probably noise spickle. Remove a very small area:
                            cv::circle(diff, max_pos, 3, Scalar(0), cv::FILLED);
                    } else if (max_val_double > 2 * _motion_thresh)
                        cv::circle(diff, max_pos, 3, Scalar(0), cv::FILLED);
                    else
                        return;
                    i++;
                    break;
                }
        }

    }
}

void TrackerManager::floodfind_and_remove(cv::Point seed_max_pos, uint8_t seed_max_val, uint8_t motion_noise, cv::Mat diff, cv::Mat motion_filtered_noise_mapL) {
    cv::Point2i bound_min_floodfill = seed_max_pos;
    cv::Point2i bound_max_floodfill = seed_max_pos;
    cv::Point2i bound_min_size = seed_max_pos;
    cv::Point2i bound_max_size = seed_max_pos;

    std::vector<cv::Point2i> todo_pts;

    int npixels = 0;
    uint32_t motion_sum = 0; // a measure for the amount of measured photons from a blob
    cv::Point2l COG = {0};

    Point min_pos, max_pos = seed_max_pos;
    double min_val;
    uint8_t max_val = seed_max_val;
    uint8_t max_diff = diff.at<uint8_t>(seed_max_pos);

    while (max_val > motion_filtered_noise_mapL.at<uint8_t>(max_pos)) {
        todo_pts.push_back(max_pos);
        while (!todo_pts.empty()) {
            auto p = todo_pts.back();
            todo_pts.pop_back();
            if (diff.at<uint8_t>(p) > motion_filtered_noise_mapL.at<uint8_t>(p)) {
                motion_sum += diff.at<uint8_t>(p) - motion_filtered_noise_mapL.at<uint8_t>(p);

                if (p.x < bound_min_floodfill.x)
                    bound_min_floodfill.x = p.x;
                else if (p.x > bound_max_floodfill.x)
                    bound_max_floodfill.x = p.x;
                if (p.y < bound_min_floodfill.y)
                    bound_min_floodfill.y = p.y;
                else if (p.y > bound_max_floodfill.y)
                    bound_max_floodfill.y = p.y;

                // For size calc, only use the pixels that are 1/3 of the max diff, based on full width half max.
                if (diff.at<uint8_t>(p) > max_diff / 3) {
                    npixels++;
                    COG.x += p.x;
                    COG.y += p.y;
                    if (p.x < bound_min_size.x)
                        bound_min_size.x = p.x;
                    else if (p.x > bound_max_size.x)
                        bound_max_size.x = p.x;
                    if (p.y < bound_min_size.y)
                        bound_min_size.y = p.y;
                    else if (p.y > bound_max_size.y)
                        bound_max_size.y = p.y;
                }

                diff.at<uint8_t>(p) = 0;

                //add pixel directly above, below or besides the max point
                if (p.x > 0)
                    todo_pts.push_back(cv::Point2i(p.x - 1, p.y));
                if (p.x < diff.cols - 1)
                    todo_pts.push_back(cv::Point2i(p.x + 1, p.y));
                if (p.y > 0)
                    todo_pts.push_back(cv::Point2i(p.x, p.y - 1));
                if (p.y < diff.rows - 1)
                    todo_pts.push_back(cv::Point2i(p.x, p.y + 1));

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
        cv::Rect bounding_box = clamp_rect(cv::Rect(bound_min_floodfill + cv::Point(-4, -4), bound_max_floodfill + cv::Point(4, 4)), diff.cols, diff.rows);
        double max_val_double;
        cv::minMaxLoc(diff(bounding_box), &min_val, &max_val_double, &min_pos, &max_pos);
        max_val = max_val_double;
        max_pos = max_pos + cv::Point(bounding_box.x, bounding_box.y);
    }
    if (!npixels)
        COG = seed_max_pos;
    else
        COG /= npixels;
    float size = normf(bound_max_size - bound_min_size) + 1.f;
    _blobs.push_back(tracking::BlobProps(COG, size, npixels, motion_sum, seed_max_val, motion_noise, _visdat->overexposed(COG), _visdat->frame_id));

    if (_enable_viz_blob) {
        bound_max_floodfill += Point(8, 8);
        bound_min_floodfill -= Point(8, 8);
        cv::Rect bounding_box = clamp_rect(cv::Rect(bound_min_floodfill, bound_max_floodfill), diff.cols, diff.rows);

        Mat viz;
        Rect bounding_box_unscaled = clamp_rect(cv::Rect(bound_min_floodfill * im_scaler, bound_max_floodfill * im_scaler), IMG_W, IMG_H);;
        cv::Mat frameL_roi = _visdat->frameL(bounding_box_unscaled);
        cv::Mat frameL_small_roi;
        cv::resize(frameL_roi, frameL_small_roi, cv::Size(frameL_roi.cols / im_scaler, frameL_roi.rows / im_scaler));

        cv::Mat overexposed_roi;
        if (_visdat->overexposed_mapL.cols) {
            cv::Mat tmp = _visdat->overexposed_mapL(bounding_box_unscaled);
            cv::resize(tmp, overexposed_roi, cv::Size(frameL_roi.cols / im_scaler, frameL_roi.rows / im_scaler));
        } else
            overexposed_roi = cv::Mat::zeros(bounding_box.size(), CV_8UC1);

        cv::Mat diff_annotated = diff(bounding_box);
        cvtColor(diff_annotated, diff_annotated, cv::COLOR_GRAY2BGR);
        cv::insertChannel(_visdat->diffL_small(bounding_box), diff_annotated, 2);

        viz = create_row_image({diff_annotated * 10, _visdat->diffL_small(bounding_box) * 10, frameL_small_roi, motion_filtered_noise_mapL(bounding_box) * 10, overexposed_roi}, CV_8UC3, viz_blobs_resizef);
        cv::Point2i COG_viz = cv::Point(COG.x - bounding_box.x, COG.y - bounding_box.y) * viz_blobs_resizef;
        cv::circle(viz, COG_viz, 1, Scalar(0, 255, 0), 2, cv::FILLED);
        vizs_blobs.push_back(viz);
    }
}

void TrackerManager::collect_static_ignores() {
    tracking::IgnoreBlob landingspot;
    for (auto &trkr : _trackers) {
        collect_static_ignores(trkr);
    }
}
void TrackerManager::collect_static_ignores(ItemTracker *trkr1) {
    tracking::IgnoreBlob landingspot;
    std::vector<tracking::IgnoreBlob> ignores;
    for (auto &trkr2 : _trackers) {
        if (trkr1->uid() != trkr2->uid()) {
            for (auto &ign : trkr2->ignores_for_other_trkrs) {
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
    [&](auto & fp) {return time - fp.last_seen_time > std::clamp(fp.detection_count * 10, 1, 60);}),
    false_positives.end());
}

void TrackerManager::match_blobs_to_trackers(double time) {

    for (auto trkr : _trackers)
        trkr->all_blobs(_blobs);

    std::vector<ProcessedBlob> pbs;
    if (_blobs.size() > 0) {
        prep_blobs(&pbs, time);
        match_existing_trackers(&pbs, time);
        if (_mode == t_locate_drone) {
            rematch_blink_tracker(&pbs, time);
            create_new_blink_trackers(&pbs, time);
        } else {
            rematch_drone_tracker(&pbs, time);
            flag_used_static_ignores(&pbs);
            create_new_insect_trackers(&pbs, time);
        }
    }

    draw_motion_viz(&pbs, time);
    draw_blob_viz(&pbs);

    for (auto trkr : _trackers)
        trkr->invalidize(false);
}
void TrackerManager::prep_blobs(std::vector<ProcessedBlob> *pbs, double time) {
    //init processed blob list and check on fp's
    uint id_cnt = 0;
    for (auto &blob : _blobs) {
        pbs->push_back(ProcessedBlob(&blob, id_cnt));
        id_cnt++;
        for (auto &fp : false_positives) {
            float dist = normf(fp.im_pt - blob.pt() * im_scaler);
            if (dist < fp.im_size) {
                blob.false_positive = true;
                fp.last_seen_time = time;
                fp.detection_count++;
            }
        }
    }
}

void TrackerManager::match_existing_trackers(std::vector<ProcessedBlob> *pbs, double time) {
    std::vector<ScorePair> scores;
    for (auto trkr : _trackers) {
        if (trkr->tracking()) {
            for (auto &blob : *pbs) {
                auto *props = blob.props;
                float score  = trkr->score(props);
                blob.tracker_scores.push_back(pair<int, float>(trkr->uid(), score));
                scores.push_back(ScorePair(score, trkr, &blob));
                if (_enable_viz_blob && trkr->viz_id() < 6) {
                    cv::Mat viz = vizs_blobs.at(blob.id);
                    cv::Point2i pt(viz.cols / 5 + 2, viz.rows - 20 * (trkr->viz_id() + 1));
                    putText(viz, "#" + std::to_string(trkr->viz_id()) + ": " + to_string_with_precision(score, 2), pt, FONT_HERSHEY_SIMPLEX, 0.4, tracker_color(trkr));
                }

            }
        }
    }

    std::sort(scores.begin(), scores.end(), [](const ScorePair & a, const ScorePair & b) -> bool { return a.score < b.score; });

    for (size_t i = 0; i < scores.size(); i++) {
        if (scores.at(i).superfluous)
            continue;
        if (scores.at(i).score < scores.at(i).tracker->score_threshold()) {
            scores.at(i).blob->trackers.push_back(scores.at(i).tracker);
            scores.at(i).tracker->calc_world_item(scores.at(i).blob->props, time);
            tracking::ImageItem image_item(*(scores.at(i).blob->props), _visdat->frame_id, scores.at(i).score, scores.at(i).blob->id);
            tracking::WorldItem w(image_item, scores.at(i).blob->props->world_props);
            scores.at(i).tracker->world_item(w);

            for (size_t j = i + 1; j < scores.size(); j++) {
                if (scores.at(j).blob->id == scores.at(i).blob->id || scores.at(j).tracker->uid() == scores.at(i).tracker->uid())
                    scores.at(j).superfluous = true;
            }
        }
    }
}

void TrackerManager::rematch_drone_tracker(std::vector<ProcessedBlob> *pbs, double time) {
    for (auto dtrkr : dronetrackers()) {
        if (!dtrkr->tracking()) {
            for (auto &blob : *pbs) {
                if (!blob.tracked() && (!blob.props->false_positive)) {

                    //check against static ignore points
                    auto props = blob.props;
                    bool in_im_ignore_zone = dtrkr->check_ignore_blobs(props);
                    if (in_im_ignore_zone)
                        blob.ignored = true;

                    if (!in_im_ignore_zone) {
                        auto score = dtrkr->score(props);
                        if (score < dtrkr->score_threshold()) {

                            dtrkr->calc_world_item(props, time);
                            tracking::WorldItem w(tracking::ImageItem(*props, _visdat->frame_id, 0, blob.id), props->world_props);
                            if (w.valid) {
                                dtrkr->world_item(w);
                                blob.trackers.push_back(dtrkr);

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
                        if (score < trkr->score_threshold()) {
                            trkr->calc_world_item(props, time);
                            tracking::WorldItem w(tracking::ImageItem(*props, _visdat->frame_id, 0, blob.id), props->world_props);
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
            for (auto &ignore : trkr->ignores_for_other_trkrs) {
                float dist_ignore = normf(ignore.p - blob.pt());
                if (dist_ignore < blob.size() + ignore.radius) {
                    ignore.was_used = true;
                }
            }
        }
    }
}
void TrackerManager::create_new_insect_trackers(std::vector<ProcessedBlob> *pbs, double time) {
    //see if there are still blobs left untracked, create new trackers for them
    if (_visdat->no_recent_brightness_events(time)) {
        for (auto &blob : *pbs) {
            auto props = blob.props;
            if (!blob.tracked() && (!props->in_overexposed_area || _mode == t_locate_drone) && (!props->false_positive || _mode == t_locate_drone) && !blob.ignored && _trackers.size() < 30) { // if so, start tracking it!
                float im_dist_to_drone = INFINITY;
                for (auto drone : dronetrackers()) {
                    if (drone->tracking()) {
                        float tmpf = normf(drone->image_item().pt() - props->pt_unscaled());
                        if (tmpf < im_dist_to_drone)
                            im_dist_to_drone = tmpf;
                    } else if (drone->image_predict_item().valid && drone->in_flight()) {
                        float tmpf = normf(drone->image_predict_item().pt - props->pt_unscaled());
                        if (tmpf < im_dist_to_drone)
                            im_dist_to_drone = tmpf;
                    }
                }

                if (im_dist_to_drone > InsectTracker::new_tracker_drone_ignore_zone_size_im && blob.pixel_max() > blob.motion_noise() + _motion_thresh) {
                    InsectTracker *it;
                    it = new InsectTracker();
                    it->init(next_insecttrkr_id, _visdat, _motion_thresh, _trackers.size(), _enable_draw_stereo_viz);
                    it->calc_world_item(props, time);
                    if (!props->world_props.bkg_check_ok)
                        tracking::WorldItem w(tracking::ImageItem(*props, _visdat->frame_id, -1, blob.id), props->world_props);

                    bool delete_it = true;
                    bool ignore = true;
                    if (props->world_props.valid) {
                        collect_static_ignores(it);
                        ignore = it->check_ignore_blobs(props);
                        blob.ignored = ignore;
                    }
                    if (!ignore) {
                        //ignore a region around the drone (or take off location)
                        float world_dist_to_drone = INFINITY;
                        for (auto drone : dronetrackers()) {
                            if (drone->tracking()) {
                                float tmpf  = normf(drone->world_item().pt - props->world_props.pt());
                                if (tmpf < world_dist_to_drone) {
                                    world_dist_to_drone = tmpf;
                                }
                            }
                        }

                        if (world_dist_to_drone > InsectTracker::new_tracker_drone_ignore_zone_size_world && im_dist_to_drone > InsectTracker::new_tracker_drone_ignore_zone_size_im) {
                            it->init_logger();
                            std::cout << "Keeping insecttracker: " << next_insecttrkr_id << " uid: " << it->uid() << std::endl;
                            next_insecttrkr_id++;
                            tracking::WorldItem w(tracking::ImageItem(*props, _visdat->frame_id, 0, blob.id), props->world_props);
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
        if (!blob.tracked() && !blob.ignored && _trackers.size() < 30) {
            bool too_close = false;
            for (auto btrkr : _trackers) {
                if (btrkr->type() == tt_blink) {
                    if (normf(btrkr->image_item().pt() - props->pt_unscaled())  < default_roi_radius * im_scaler) {
                        too_close = true;
                    }
                }
            }

            if (!too_close) {
                BlinkTracker  *bt;
                bt = new BlinkTracker();
                bt->init(next_blinktrkr_id, _visdat, _motion_thresh, _trackers.size());
                bt->calc_world_item(props, time);
                bool ignore = true;
                if (props->world_props.valid) {
                    collect_static_ignores(bt);
                    ignore = bt->check_ignore_blobs(props);
                }
                if (!ignore) {
                    bt->init_logger();
                    next_blinktrkr_id++;
                    tracking::WorldItem w(tracking::ImageItem(*props, _visdat->frame_id, 0, blob.id), props->world_props);
                    bt->world_item(w);
                    _trackers.push_back(bt);
                    blob.trackers.push_back(bt);
                } else {
                    bt->close();
                    delete bt;
                }
            }
        }
    }
}
void TrackerManager::update_trackers(double time, long long frame_number) {
    //perform all tracker update functions or logging placeholders, also delete old trackers
    for (uint ii = _trackers.size(); ii != 0; ii--) { // reverse because deleting from this list.
        uint i = ii - 1;
        if (_trackers.at(i)->delete_me()) {
            ItemTracker *trkr = _trackers.at(i);
            if (trkr->type() == tt_insect) {
                InsectTracker *itrkr = static_cast<InsectTracker *>(_trackers.at(i));
                auto fpt = itrkr->false_positive();
                if (fpt == fp_static_location && itrkr->track().size() && !_monster_alert) {
                    _fp_statics_count++;
                    false_positives.push_back(FalsePositive(itrkr->track().begin()->world_item, fpt, time));
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
        } else if (_trackers.at(i)->type() == tt_drone) {
            DroneTracker *dtrkr = static_cast<DroneTracker *>(_trackers.at(i));
            dtrkr->update(time);
        } else if (_trackers.at(i)->type() == tt_insect) {
            InsectTracker *itrkr = static_cast<InsectTracker *>(_trackers.at(i));
            switch (_mode) {
                case t_idle: {
                        itrkr->append_log(time, frame_number); // write dummy data
                        break;
                } case t_locate_drone: {
                        itrkr->append_log(time, frame_number); // write dummy data
                        break;
                } case t_c: {
                        itrkr->update(time);
                        break;
                } case t_x: {
                        itrkr->update(time);
                        break;
                    }
            }
            if (itrkr->monster_alert())
                time_since_monsters = time;
        } else if (_trackers.at(i)->type() == tt_replay) {
            ReplayTracker *rtrkr = static_cast<ReplayTracker *>(_trackers.at(i));
            rtrkr->update(time);
        } else if (_trackers.at(i)->type() == tt_virtualmoth) {
            VirtualMothTracker *vtrkr = static_cast<VirtualMothTracker *>(_trackers.at(i));
            vtrkr->update(time);
        } else if (_trackers.at(i)->type() == tt_blink) {
            BlinkTracker *btrkr = static_cast<BlinkTracker *>(_trackers.at(i));
            btrkr->update(time);
            if (_mode == t_locate_drone) {
                if (btrkr->state() == BlinkTracker::bds_found) {
                    _detected_blink_locations.push_back(btrkr->world_item().pt);
                }
            } else if (btrkr->ignores_for_other_trkrs.size() == 0) {
                if (btrkr->state() != BlinkTracker::bds_found) {
                    double v = 0;
                    if (btrkr->last_track_data().vel_valid)
                        v = norm(btrkr->last_track_data().vel());
                    if (v < 0.2)
                        _visdat->delete_from_motion_map(btrkr->image_item().pt(), btrkr->image_item().disparity, btrkr->image_item().size + 5, 1);
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
        cv::cvtColor(_visdat->diffL * 10, diff_viz, cv::COLOR_GRAY2BGR);
    reset_trkr_viz_ids();
}
void TrackerManager::draw_trackers_viz() {
    if (_enable_viz_tracker) {
        int h = 0;
        cv::Scalar white = cv::Scalar(255, 255, 255);
        cv::Scalar red = cv::Scalar(0, 0, 255);
        int size_text = 12;
        int descr_w = 320;
        int descr_h = 10 * size_text; // -> lines of text in description frame
        int req_h = 0, req_w = descr_w;
        for (auto trkr : _trackers) { //dry run to figure out size of the final viz
            auto image_item = trkr->image_item();
            if (image_item.valid) {
                auto [roiL, roiR, resize_factor] = calc_trkr_viz_roi(image_item);
                int height = roiL.height;
                if (roiR.height > height)
                    height = roiR.height;
                int width = roiL.width;
                if (roiR.width > width)
                    width = roiR.width;
                if (height * 2 * resize_factor > descr_h)
                    req_h += height * 2 * resize_factor;
                else
                    req_h += descr_h;
                if (width * 4 * resize_factor + descr_w > req_w)
                    req_w = width * 4 * resize_factor + descr_w;
            } else
                req_h += descr_h;
        }

        cv::Mat viz_tracker = cv::Mat::zeros(cv::Size(req_w, req_h), CV_8UC3);
        for (auto trkr : _trackers) {
            auto image_item = trkr->image_item();
            auto wti = trkr->world_item();
            cv::Rect roiD(0, h, descr_w, descr_h);
            cv::Mat viz_descr =  viz_tracker(roiD);
            int y_text = 1;
            putText(viz_descr, "Tracker " + std::to_string(trkr->viz_id()) + ": " + trkr->name(), cv::Point2i(1, y_text++ * size_text), FONT_HERSHEY_DUPLEX, 0.4, white);

            if (image_item.valid) {
                auto [roiL, roiR, resize_factor] = calc_trkr_viz_roi(image_item);

                std::vector<cv::Mat> vizsL;
                vizsL.push_back(_visdat->frameL(roiL));
                vizsL.push_back(_visdat->diffL(roiL) * 10);
                if (_visdat->motion_filtered_noise_mapL.cols)
                    vizsL.push_back(_visdat->motion_filtered_noise_mapL(roiL) * 10);
                if (_visdat->overexposed_mapL.cols)
                    vizsL.push_back(_visdat->overexposed_mapL(roiL));
                cv::Mat vizL = create_row_image(vizsL, CV_8UC3, 1);

                std::vector<cv::Mat> vizsR;
                vizsR.push_back(_visdat->frameR(roiR));
                vizsR.push_back(_visdat->diffR(roiR) * 10);
                if (_visdat->motion_filtered_noise_mapR.cols)
                    vizsR.push_back(_visdat->motion_filtered_noise_mapR(roiR) * 10);
                if (_visdat->overexposed_mapR.cols)
                    vizsR.push_back(_visdat->overexposed_mapR(roiR));
                cv::Mat vizR = create_row_image(vizsR, CV_8UC3, 1);

                cv::Mat vizLR = create_column_image({vizL, vizR}, CV_8UC3, resize_factor);

                cv::Rect roiLR(roiD.width, h, vizLR.cols, vizLR.rows);
                vizLR.copyTo(viz_tracker(roiLR));
                if (vizLR.rows > roiD.height)
                    h += vizLR.rows;
                else
                    h += roiD.height;

                putText(viz_descr, "blob id -> tr_uid: " + std::to_string(image_item.blob_id) + " -> " + std::to_string(trkr->uid()), cv::Point2i(1, y_text++ * size_text), FONT_HERSHEY_SIMPLEX, 0.4, white);
                putText(viz_descr, std::to_string(resize_factor) + "x", cv::Point2i(viz_descr.cols - 16, size_text), FONT_HERSHEY_SIMPLEX, 0.4, red);
                putText(viz_descr, "Score: " + to_string_with_precision(image_item.score, 2), cv::Point2i(1, y_text++ * size_text), FONT_HERSHEY_SIMPLEX, 0.4, white);
                putText(viz_descr, "# frames: " + std::to_string(trkr->n_frames_tracking()) + " / " + std::to_string(trkr->n_frames()), cv::Point2i(1, y_text++ * size_text), FONT_HERSHEY_SIMPLEX, 0.4, white);
                putText(viz_descr, "Size: " + to_string_with_precision(image_item.size, 1), cv::Point2i(1, y_text++ * size_text), FONT_HERSHEY_SIMPLEX, 0.4, white);
                putText(viz_descr, "Max: " + to_string_with_precision(image_item.pixel_max, 1), cv::Point2i(1, y_text++ * size_text), FONT_HERSHEY_SIMPLEX, 0.4, white);
                putText(viz_descr, "i: [" + to_string_with_precision(image_item.x, 1) + ", " + to_string_with_precision(image_item.y, 1) + "], " +  to_string_with_precision(image_item.disparity, 2), cv::Point2i(1, y_text++ * size_text), FONT_HERSHEY_SIMPLEX, 0.4, white);
                putText(viz_descr, "w: [" + to_string_with_precision(wti.pt.x, 2) + ", " + to_string_with_precision(wti.pt.y, 2) + ", " + to_string_with_precision(wti.pt.z, 2) + "] ||" + to_string_with_precision(wti.radius, 3) + "||", cv::Point2i(1, y_text++ * size_text), FONT_HERSHEY_SIMPLEX, 0.4, white);

                int text_w = 0;
                if (trkr->image_predict_item().out_of_image) {
                    std::string flag = "OUT OF IMAGE";
                    putText(viz_descr, flag, cv::Point2i(1, y_text++ * size_text), FONT_HERSHEY_TRIPLEX, 0.4, cv::Scalar(0, 0, 255));
                    int baseline;
                    Size text_size = getTextSize(flag, FONT_HERSHEY_TRIPLEX, 0.75, 2, &baseline);
                    text_w += text_size.width;
                } else if (trkr->image_predict_item().out_of_image_right) {
                    std::string flag = "OUT OF RIGHT IMAGE";
                    putText(viz_descr, flag, cv::Point2i(1, y_text++ * size_text), FONT_HERSHEY_TRIPLEX, 0.4, cv::Scalar(0, 0, 255));
                    int baseline;
                    Size text_size = getTextSize(flag, FONT_HERSHEY_TRIPLEX, 0.75, 2, &baseline);
                    text_w += text_size.width;
                }
                if (trkr->type() == tt_insect) {
                    InsectTracker *itrkr = static_cast<InsectTracker *>(trkr);
                    if (itrkr->false_positive()) {
                        std::string flag = "";
                        if (text_w > 0)
                            flag += " | ";
                        flag += false_positive_names[itrkr->false_positive()];
                        putText(viz_descr, flag, cv::Point2i(1, y_text++ * size_text), FONT_HERSHEY_TRIPLEX, 0.4, cv::Scalar(255, 0, 0));
                        int baseline;
                        Size text_size = getTextSize(flag, FONT_HERSHEY_TRIPLEX, 0.75, 2, &baseline);
                        text_w += text_size.width;
                    }
                }
                if (image_item.blob_is_fused) {
                    std::string flag = "";
                    if (text_w > 0)
                        flag += " | ";
                    flag += "Fused";
                    putText(viz_descr, flag, cv::Point2i(1, y_text++ * size_text), FONT_HERSHEY_TRIPLEX, 0.4, cv::Scalar(0, 128, 255));
                    int baseline;
                    Size text_size = getTextSize(flag, FONT_HERSHEY_TRIPLEX, 0.75, 2, &baseline);
                    text_w += text_size.width;
                }

            } else {
                putText(viz_descr, "No blob found", cv::Point2i(1, y_text++ * size_text), FONT_HERSHEY_SIMPLEX, 0.4, red);
                putText(viz_descr, "tr_uid: " + std::to_string(trkr->uid()), cv::Point2i(1, y_text++ * size_text), FONT_HERSHEY_SIMPLEX, 0.4, white);
                putText(viz_descr, "Lost: " + std::to_string(trkr->n_frames_lost()) + " / " + std::to_string(trkr->n_frames()), cv::Point2i(1, y_text++ * size_text), FONT_HERSHEY_SIMPLEX, 0.4, white);
                int text_w = 0;
                if (trkr->image_predict_item().out_of_image) {
                    std::string flag = "OUT OF IMAGE";
                    putText(viz_descr, flag, cv::Point2i(1, y_text++ * size_text), FONT_HERSHEY_TRIPLEX, 0.4, cv::Scalar(0, 0, 255));
                    int baseline;
                    Size text_size = getTextSize(flag, FONT_HERSHEY_TRIPLEX, 0.75, 2, &baseline);
                    text_w += text_size.width;
                }
                if (trkr->type() == tt_insect) {
                    InsectTracker *itrkr = static_cast<InsectTracker *>(trkr);
                    if (itrkr->false_positive()) {
                        std::string flag = "";
                        if (text_w > 0)
                            flag += " | ";
                        flag += false_positive_names[itrkr->false_positive()];
                        putText(viz_descr, flag, cv::Point2i(1, y_text++ * size_text), FONT_HERSHEY_TRIPLEX, 0.4, cv::Scalar(255, 0, 0));
                        int baseline;
                        Size text_size = getTextSize(flag, FONT_HERSHEY_TRIPLEX, 0.75, 2, &baseline);
                        text_w += text_size.width;
                    }
                }
                h += roiD.height;
            }
        }
        if (!viz_tracker.rows || !viz_tracker.cols)
            viz_tracker = cv::Mat::zeros(5, 100, CV_8UC3);
        viz_trkrs_buf = viz_tracker.clone();
    }
}
void TrackerManager::draw_motion_viz(std::vector<ProcessedBlob> *pbs, double time) {
    if (enable_viz_motion) {
        for (auto blob : *pbs) {
            std::string s = std::to_string(blob.id);
            if (blob.trackers.size() > 0) {
                s = s + "->";
                for (auto trkr : blob.trackers) {
                    s = s + std::to_string(trkr->uid()) + "+";
                }
                s.pop_back();
            }
            cv::Point target_viz_p = (blob.props->pt_unscaled() + cv::Point2f(10, -10));
            putText(diff_viz, blob.prefix() +  s, target_viz_p, FONT_HERSHEY_SIMPLEX, 0.4, blob.color(), 2);
            cv::line(diff_viz, target_viz_p, blob.props->pt_unscaled(), blob.color());

        }
        for (auto trkr : replaytrackers()) {
            cv::Point target_viz_p = (trkr->image_item().pt() + cv::Point2f(10, -10));
            putText(diff_viz, " r", target_viz_p, FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 180), 2);
            cv::line(diff_viz, target_viz_p, trkr->image_item().pt(), cv::Scalar(0, 0, 180));
        }
        for (auto trkr : virtualmothtrackers()) {
            cv::Point target_viz_p = (trkr->image_item().pt() + cv::Point2f(10, -10));
            putText(diff_viz, " v", target_viz_p, FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 180), 2);
            cv::line(diff_viz, target_viz_p, trkr->image_item().pt(), cv::Scalar(0, 0, 180));
        }

        auto blue = cv::Scalar(255, 0, 0);
        for (auto fp : false_positives) {
            auto p_center = fp.im_pt;
            cv::circle(diff_viz, p_center, fp.im_size, blue);
            cv::putText(diff_viz, to_string_with_precision(time - fp.last_seen_time, 1) + ", #" + std::to_string(fp.detection_count), p_center + cv::Point2f(10, 0), FONT_HERSHEY_SIMPLEX, 0.4, blue, 2);
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
                    msg_str = msg_str + " exp.";
                } else  if (!wblob.im_pos_ok && !blob.tracked())
                    msg_str = msg_str + " pre.";
                else {
                    if (!wblob.disparity_in_range)
                        msg_str = msg_str + " dsp.";
                    if (!wblob.bkg_check_ok)
                        msg_str = msg_str + " bkg.";

                    if (!wblob.radius_in_range)
                        msg_str = msg_str + " r.";
                    if (blob.props->ignores.size() > 0)
                        msg_str = msg_str + " ign.";
                    if (wblob.takeoff_reject)
                        msg_str = msg_str + " tko.";
                }
                putText(viz, msg_str, cv::Point2i(viz.cols / 5 + 2, viz.rows - 8), FONT_HERSHEY_SIMPLEX, 0.4, blob.color());
            }
            if (wblob.valid) {
                std::string coor_str = "[" + to_string_with_precision(wblob.x, 2) + ", " + to_string_with_precision(wblob.y, 2) + ", " + to_string_with_precision(wblob.z, 2) + "]";
                putText(viz, coor_str, cv::Point2i(viz.cols / 5 + 2, viz.rows - 8), FONT_HERSHEY_SIMPLEX, 0.4, blob.color());
            }
        }
    }
}
std::tuple<Rect, Rect, int> TrackerManager::calc_trkr_viz_roi(ImageItem image_item) {
    cv::Rect roiL(image_item.x - image_item.size - 5, image_item.y - image_item.size - 5, image_item.size * 2 + 10, image_item.size * 2 + 10);
    roiL = clamp_rect(roiL, IMG_W, IMG_H);
    cv::Rect roiR(image_item.x - image_item.disparity - image_item.size - 5, image_item.y - image_item.size - 5, image_item.size * 2 + 10, image_item.size * 2 + 10);
    roiR = clamp_rect(roiR, IMG_W, IMG_H);
    int resize_factor = 1 ;
    if (roiL.height < 15)
        resize_factor = 4;
    else if (roiL.height < 30)
        resize_factor = 2;

    return std::make_tuple(roiL, roiR, resize_factor);
}
void TrackerManager::finish_vizs() {
    if (_enable_viz_blob && vizs_blobs.size() > 0)
        viz_max_points = create_column_image(vizs_blobs, CV_8UC3, 1);
    else if (_enable_viz_blob)
        viz_max_points = cv::Mat::zeros(5, 100, CV_8UC3);
    if (enable_viz_motion)
        diff_viz_buf = diff_viz.clone();;
}
void TrackerManager::reset_trkr_viz_ids() {
    int16_t cnt = 0;
    for (auto trkr : _trackers)
        trkr->viz_id(cnt++);
}

bool TrackerManager::tracker_active(ItemTracker *trkr) {
    if (_mode == t_idle)
        return false;
    else if (trkr->type() == tt_blink && (_mode != t_locate_drone)) {
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
std::vector<DroneTracker *> TrackerManager::dronetrackers() {
    std::vector<DroneTracker *> res;
    for (auto trkr : _trackers) {
        if (trkr->type() == tt_drone) {
            res.push_back(static_cast<DroneTracker *>(trkr));
        }
    }
    //should do the same:
    //std::copy_if (_trackers.begin(), _trackers.end(), std::back_inserter(res), [](ItemTracker *trkr){return (trkr->type == tt_insect);} );
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
    return std::make_tuple(valid, best_btrkr);
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
            throw std::runtime_error("Cannot read: " + settings_file);
        }
        TrackerManagerParameters tmp;
        auto v1 = params.getVersion();
        auto v2 = tmp.getVersion();
        if (v1 != v2) {
            throw std::runtime_error("XML version difference detected from " + settings_file);
        }
        infile.close();
    } else {
        throw std::runtime_error("File not found: " + settings_file);
    }

    max_points_per_frame = params.max_points_per_frame.value();
    _motion_thresh = params.motion_thresh.value();
}

void TrackerManager::serialize_settings() {
    TrackerManagerParameters params;
    params.max_points_per_frame = max_points_per_frame;
    params.motion_thresh = _motion_thresh;

    std::string xmlData = params.toXML();
    std::ofstream outfile = std::ofstream(settings_file);
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
                FalsePositive fp(static_cast<false_positive_type>(stoi(splitted_line.at(0))), stoi(splitted_line.at(2)), stof(splitted_line.at(3)), stof(splitted_line.at(4)), stof(splitted_line.at(5)));
                false_positives.push_back(fp);
            }

            save_false_positives(data_output_dir + "initial_" + false_positive_fn);
        } catch (exception &exp) {
            throw std::runtime_error("Warning: could not read false positives file: " + false_positive_rfn + '\n' + "Line: " + string(exp.what()) + " at: " + line);
        }
    }
}
void TrackerManager::save_false_positives() {
    save_false_positives(data_output_dir + false_positive_fn);
    if (replay_dir == "")
        save_false_positives("../" + false_positive_fn);
}
void TrackerManager::save_false_positives(string false_positive_wfn) {
    std::ofstream fp_writer;
    fp_writer.open(false_positive_wfn, std::ofstream::out);
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

void TrackerManager::close() {
    if (initialized) {
        for (auto trkr : _trackers)
            trkr->close();
        save_false_positives();
        initialized = false;

    }
}

cv::Scalar TrackerManager::color_of_blob(ProcessedBlob blob) {
    if (blob.trackers.size() == 0) {
        if (blob.ignored)
            return cv::Scalar(0, 128, 0); // dark green
        else
            return cv::Scalar(255, 255, 55); // light blue
    } else if (blob.trackers.size() > 1)
        return cv::Scalar(200, 255, 250);
    ItemTracker *trkr = blob.trackers.at(0);
    if (trkr->type() == tt_drone)
        return cv::Scalar(0, 255, 0); // green
    else if (trkr->type() == tt_insect)
        return cv::Scalar(0, 0, 255); // red
    else if (trkr->type() == tt_replay)
        return cv::Scalar(0, 0, 180); // dark red
    else if (trkr->type() == tt_virtualmoth)
        return cv::Scalar(255, 0, 255); // pink
    return cv::Scalar(0, 0, 0);
}

}
