#include "trackermanager.h"
using namespace cv;
using namespace std;

void TrackerManager::init(std::ofstream *logger,VisionData *visdat){
    _visdat = visdat;
    _logger = logger;

    enable_viz_max_points = false;
    enable_viz_diff = true;

    if (pparams.video_cuts)
        enable_viz_diff = true;

    deserialize_settings();

    //TODO: update comment below, fixed?
    //we have a bit of a situation with the order of initializing the trackers, as this MUST be in the same order as how they are called
    //in the future this is prolly not sustainable, as we have multiple insect trackers etc
    //ATM the blinktracker is therefor not logged, as there can be multiple and they disappear after they are done
    //Anyway, the track() functions are called in a reverse for loop (must be reversed because items are erased from the list),
    //therefor the init functions must be called in the reverser order as adding them to the _trackers list...
    //...but since we are 'pushing' the items in, this in the end must be in the normal order. Or not. I guess it's a 50% chance thing.
    //#106
    _dtrkr = new DroneTracker();
    default_itrkr = new InsectTracker();
    _dtrkr->init(_logger,_visdat);
    default_itrkr->init(0,_visdat);
    _trackers.push_back(default_itrkr);
    _trackers.push_back(_dtrkr);

    (*_logger) << "trkrs_state;";
    initialized = true;
}

void TrackerManager::update(double time, bool drone_is_active){

    if (enable_viz_diff) {
        cv::cvtColor(_visdat->diffL*10,diff_viz,CV_GRAY2BGR);
    }

    if (_mode != mode_idle){
        update_max_change_points();
        update_static_ignores();
        match_blobs_to_trackers(drone_is_active && !_dtrkr->inactive(),time);
    }

    update_trackers(time,_visdat->frame_id, drone_is_active);

    (*_logger) << static_cast<int16_t>(_mode) << ";";

    if (enable_viz_max_points && vizs_maxs.size()>0)
        viz_max_points = create_column_image(vizs_maxs,CV_8UC3,1);
    else if(enable_viz_max_points)
        viz_max_points = cv::Mat::zeros(5,100,CV_8UC3);
}

void TrackerManager::update_trackers(double time,long long frame_number, bool drone_is_active) {
    //perform all tracker functions, also delete old trackers
    for (uint ii=_trackers.size();ii != 0; ii--){ // reverse because deleting from this list.
        uint i = ii-1;
        if (_trackers.at(i)->delete_me()){
            ItemTracker * trkr = _trackers.at(i);
            _trackers.erase(_trackers.begin() + i);
            delete trkr;
        } else if(typeid(*_trackers.at(i)) == typeid(DroneTracker)) {
            DroneTracker * dtrkr = static_cast<DroneTracker * >(_trackers.at(i));
            dtrkr->track(time,tracker_active(dtrkr,drone_is_active));
        } else if (typeid(*_trackers.at(i)) == typeid(InsectTracker)) {
            InsectTracker * itrkr = static_cast<InsectTracker * >(_trackers.at(i));
            switch (_mode){
            case mode_idle:{
                itrkr->append_log(time,frame_number); // write dummy data
                break;
            } case mode_locate_drone:{
                itrkr->append_log(time,frame_number); // write dummy data
                break;
            } case mode_wait_for_insect:{
                itrkr->track(time);
                break;
            } case mode_hunt:{
                itrkr->track(time);
                break;
            } case mode_drone_only:{
                itrkr->append_log(time,frame_number); // write dummy data
                break;
            }
            }
        } else if(typeid(*_trackers.at(i)) == typeid(ReplayTracker)) {
            ReplayTracker * rtrkr = static_cast<ReplayTracker * >(_trackers.at(i));
            rtrkr->track(time);
        } else if (typeid(*_trackers.at(i)) == typeid(BlinkTracker)) {
            BlinkTracker * btrkr = static_cast<BlinkTracker * >(_trackers.at(i));
            btrkr->track(time);
            if (_mode == mode_locate_drone) {
                if (btrkr->state() == BlinkTracker::bds_found){
                    _dtrkr->set_drone_landing_location(btrkr->image_item().pt(),btrkr->world_item().iti.disparity,btrkr->smoothed_size_image(),btrkr->world_item().pt);
                    _mode = mode_wait_for_insect;
                }
            } else if (btrkr->ignores_for_other_trkrs.size() == 0){
                if (btrkr->state() != BlinkTracker::bds_found) {
                    double v = 0;
                    if (btrkr->Last_track_data().vel_valid)
                        v = norm(btrkr->Last_track_data().vel());
                    if (v < 0.2)
                        _visdat->delete_from_motion_map(btrkr->image_item().pt()*pparams.imscalef,btrkr->image_item().disparity,btrkr->image_item().size*pparams.imscalef+5,1);
                }

                _trackers.erase(_trackers.begin() + i);
                delete btrkr;
            }
        }
    }
}

//for each tracker collect the static ignores from each other tracker
void TrackerManager::update_static_ignores() {
    bool landingspot_ignore_found = false;
    ItemTracker::IgnoreBlob landingspot;
    for (uint i=0; i<_trackers.size();i++){
        std::vector<ItemTracker::IgnoreBlob> ignores;
        for (uint j=0; j<_trackers.size();j++){
            if (j!=i){
                for (uint k = 0; k < _trackers.at(j)->ignores_for_other_trkrs.size();k++){
                    ignores.push_back(_trackers.at(j)->ignores_for_other_trkrs.at(k));
                    if (_trackers.at(j)->ignores_for_other_trkrs.at(k).ignore_type == ItemTracker::IgnoreBlob::takeoff_spot){
                        landingspot_ignore_found = true;
                        landingspot = _trackers.at(j)->ignores_for_other_trkrs.at(k);
                    }
                }
            }
        }
        _trackers.at(i)->ignores_for_me = ignores;
    }
    if (landingspot_ignore_found) // tmp solution until landingspot itemtracker is made
        _dtrkr->ignores_for_me.push_back(landingspot);
}

void TrackerManager::match_blobs_to_trackers(bool drone_is_active, double time) {

    //set all trackers to invalid so we can use this as a flag to notice when tracking is lost.
    for (auto trkr : _trackers)
        trkr->item_invalidize();

    if (_blobs.size()>0) {
        //init keypoints list
        std::vector<processed_blobs> pbs;
        for (auto & blob : _blobs)
            pbs.push_back(processed_blobs(&blob));

        //first check if there are trackers that were already tracking something, which prediction matches a new keypoint
        for (uint i=0; i<_trackers.size();i++){
            ItemTracker * trkr = _trackers.at(i);
            float best_score = -1;
            int best_score_j;

            if (tracker_active(trkr,drone_is_active) && trkr->tracking()) {
                for (uint j=0; j < pbs.size(); j++) {
                    float score =0;
                    //check against static ignore points
                    ItemTracker::BlobProps *tmp = pbs.at(j).props;
                    bool in_ignore_zone = trkr->check_ignore_blobs(tmp,time);
                    if (in_ignore_zone)
                        pbs.at(j).ignored = true;
                    if (!in_ignore_zone) {
                        score  = trkr->score(*tmp);
                        if (score > best_score){
                            best_score = score;
                            best_score_j = j;
                        }
                        if (enable_viz_max_points) {
                            cv::Mat viz = vizs_maxs.at(j);
                            cv::Point2i pt(viz.cols-50,viz.rows - 14*(i+1));
                            putText(viz,to_string_with_precision(score,1),pt,FONT_HERSHEY_SIMPLEX,0.5,tracker_color(trkr));
                        }
                    } else if (enable_viz_max_points){
                        cv::Mat viz = vizs_maxs.at(j);
                        cv::Point2i pt(viz.cols-50,viz.rows - 14*(i+1));
                        putText(viz,"Ign.",pt,FONT_HERSHEY_SIMPLEX,0.5,tracker_color(trkr));
                    }
                }
                if (best_score >= trkr->score_threshold() ) {
                    auto blob = &_blobs.at(best_score_j);
                    trkr->calc_world_item(blob,time);
                    ItemTracker::WorldItem w(ItemTracker::ImageItem(_blobs.at(best_score_j),blob->world_props.disparity,_visdat->frame_id,best_score,best_score_j),blob->world_props);
                    trkr->world_item(w);
                    pbs.at(best_score_j).trackers.push_back(trkr);
                }
            }
        }

        //see if there are trackers that are not tracking yet and if there are untracked points left, which can be bound together.
        for (uint i=0; i<_trackers.size();i++){
            ItemTracker * trkr = _trackers.at(i);
            if (!trkr->tracking()) {
                if (tracker_active(trkr, drone_is_active)) {
                    for (uint j=0; j < pbs.size(); j++) {
                        if (!pbs.at(j).tracked()){

                            //check against static ignore points
                            auto blob = pbs.at(j).props;
                            bool in_im_ignore_zone = trkr->check_ignore_blobs(blob,j);
                            if (in_im_ignore_zone)
                                pbs.at(j).ignored = true;

                            if (!in_im_ignore_zone) {
                                //Check if this blob may be some residual drone motion
                                //If the insect was lost, it is unlikely that it or another insect pops up
                                //very close to the drone. So ignore it.
                                if (typeid(*trkr) == typeid(InsectTracker) && _dtrkr->tracking()) {
                                    float drone_score = _dtrkr->score(*blob);
                                    if (drone_score > _dtrkr->score_threshold())
                                        in_im_ignore_zone = true;
                                }
                            }

                            if (!in_im_ignore_zone) {
                                //The tracker has lost the item, or is still initializing.
                                //there is no valid prediction, the score is therefor as low as possible
                                trkr->calc_world_item(blob,time);
                                ItemTracker::WorldItem w(ItemTracker::ImageItem(*blob,blob->world_props.disparity,_visdat->frame_id,0,j),blob->world_props);
                                if (w.valid) {
                                    trkr->world_item(w);
                                    pbs.at(j).trackers.push_back(trkr);

                                    if (enable_viz_max_points) {
                                        cv::Mat viz = vizs_maxs.at(j);
                                        cv::Point2i pt(viz.cols-50,viz.rows - 14*(i+1));
                                        putText(viz,"New",pt,FONT_HERSHEY_SIMPLEX,0.5,tracker_color(trkr));
                                    }

                                    // There may be other interesting blobs to consider but since there is no
                                    // history available at this point, its hard to calculate which one would
                                    // be better. So just pick the first one...:
                                    break;
                                }
                            } else {
                                if (enable_viz_max_points) {
                                    cv::Mat viz = vizs_maxs.at(j);
                                    cv::Point2i pt(viz.cols-50,viz.rows - 14*(i+1));
                                    putText(viz,"Ign.",pt,FONT_HERSHEY_SIMPLEX,0.5,tracker_color(trkr));
                                }
                            }
                        } else {
                            if (enable_viz_max_points) {
                                cv::Mat viz = vizs_maxs.at(j);
                                cv::Point2i pt(viz.cols-50,viz.rows - 14*(i+1));
                                putText(viz,"Trkd.",pt,FONT_HERSHEY_SIMPLEX,0.5,tracker_color(trkr));
                            }
                        }
                    }
                }
            }
        }

        //see if there are static ignore points that are detected. If so set was_used flag
        for (auto trkr : _trackers) {
            for (auto blob : pbs) {
                for (auto& ignore : trkr->ignores_for_other_trkrs){
                    cv::Point2f p_ignore = ignore.p;
                    float dist_ignore = sqrtf(powf(p_ignore.x-blob.props->x,2)+powf(p_ignore.y-blob.props->y,2));
                    if (dist_ignore < blob.size() + ignore.radius ){
                        ignore.was_used = true;
                    }
                }
            }
        }

        //see if there are still keypoints left untracked, create new trackers for them
        for (uint i=0; i < pbs.size(); i++){
            auto blob = pbs.at(i).props;
            if (!pbs.at(i).tracked() && !pbs.at(i).ignored && _trackers.size() < 30) { // if so, start tracking it!
                if (_mode == mode_locate_drone){
                    BlinkTracker  * bt;
                    bt = new BlinkTracker();
                    bt->init(_visdat);
                    bt->calc_world_item(blob,time);
                    if (blob->world_props.valid) {
                        ItemTracker::WorldItem w(ItemTracker::ImageItem(*blob,blob->world_props.disparity,_visdat->frame_id,100,i),blob->world_props);
                        bt->world_item(w);
                        _trackers.push_back( bt);
                        pbs.at(i).trackers.push_back(bt);
                    }
                } else if (_mode != mode_idle && _mode != mode_drone_only){
                    default_itrkr->calc_world_item(blob,time);

                    if (blob->world_props.valid) {
                        //ignore a region around the drone (or take of location)
                        float dist_to_drone;
                        if (dronetracker()->tracking()){
                            dist_to_drone = normf(dronetracker()->world_item().pt- blob->world_props.pt());
                        } else {
                            dist_to_drone = normf(dronetracker()->drone_startup_location() - blob->world_props.pt());
                        }
                        if (dist_to_drone < InsectTracker::new_tracker_drone_ignore_zone_size) {
                            InsectTracker *it;
                            it = new InsectTracker();
                            it->init(next_insecttrkr_id,_visdat);
                            next_insecttrkr_id++;
                            default_itrkr->calc_world_item(blob,time);
                            ItemTracker::WorldItem w(ItemTracker::ImageItem(*blob,blob->world_props.disparity,_visdat->frame_id,100,i),blob->world_props);
                            it->world_item(w);
                            _trackers.push_back( it);
                            pbs.at(i).trackers.push_back(it);
                        } else {
                            pbs.at(i).ignored = true;
                        }
                    }
                }
            }
        }

        //check if there are conflicts (blobs having multiple trackers attached):
        //Since the insect / drone trackers will attach to the closest possible blob, if
        //the one of the items becomes invisible for whatever reason, it will get
        //attached to the other blob.
        for (uint i = 0; i < pbs.size();i++){
            if (pbs.at(i).trackers.size()> 1) {
                //see if we have a drone / insect tracker pair
                DroneTracker * dtrkr;
                InsectTracker * itrkr;
                bool drn_trkr_fnd = false;
                for (auto trkr : pbs.at(i).trackers) {
                    if (typeid(*trkr) == typeid(DroneTracker)) {
                        dtrkr = static_cast<DroneTracker *>(trkr);
                        drn_trkr_fnd = true;
                        break;
                    }
                }

                if (drn_trkr_fnd) {
                    auto blobi = &_blobs.at(i);
                    for (auto trkr : pbs.at(i).trackers) {
                        if (typeid(*trkr) == typeid(InsectTracker)) {
                            itrkr = static_cast<InsectTracker*>(trkr);

                            //so, there should be another blob close to the currently one used.
                            //if there is, the small one is prolly the moth...
                            bool conflict_resolved = false;
                            for (uint j = 0; j < pbs.size();j++){
                                if (i!=j && !pbs.at(j).tracked()){
                                    auto blobj = &_blobs.at(j);
                                    float dist = cv::norm(pbs.at(i).pt()-pbs.at(j).pt());
                                    if (dist < 2.f* (pbs.at(i).size() + pbs.at(j).size())) {
                                        if(pbs.at(i).size() > pbs.at(j).size()) {
                                            dtrkr->calc_world_item(blobi,time);
                                            ItemTracker::WorldItem wi(ItemTracker::ImageItem(*blobi,blobi->world_props.disparity,_visdat->frame_id,0,i),blobi->world_props);
                                            dtrkr->world_item(wi);

                                            itrkr->calc_world_item(blobj,time);
                                            ItemTracker::WorldItem wj(ItemTracker::ImageItem(*blobj,blobj->world_props.disparity,_visdat->frame_id,0,j),blobj->world_props);
                                            itrkr->world_item(wj);

                                            pbs.at(i).trackers.clear();
                                            pbs.at(j).trackers.clear();
                                            pbs.at(i).trackers.push_back(dtrkr);
                                            pbs.at(j).trackers.push_back(itrkr);
                                        }else {
                                            itrkr->calc_world_item(blobi,time);
                                            ItemTracker::WorldItem wi(ItemTracker::ImageItem(*blobi,blobi->world_props.disparity,_visdat->frame_id,0,i),blobi->world_props);
                                            itrkr->world_item(wi);
                                            dtrkr->calc_world_item(blobj,time);
                                            ItemTracker::WorldItem wj(ItemTracker::ImageItem(*blobj,blobj->world_props.disparity,_visdat->frame_id,0,j),blobj->world_props);
                                            dtrkr->world_item(wj);

                                            pbs.at(i).trackers.clear();
                                            pbs.at(j).trackers.clear();
                                            pbs.at(j).trackers.push_back(dtrkr);
                                            pbs.at(i).trackers.push_back(itrkr);
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
                                // hmm, apparantely there is no blob close by, so now there are two possibilities:
                                //1. The insect is lost (e.g. too far, too small, into the flowers)
                                //2. The drone is lost (e.g. crashed)
                                //3. The insect and drone are too close to eachother to distinguish
                                //4. We have a kill :) In the last few cases this resulted in a confetti storm. Maybe this can be detected.
                            }
                        }
                    }
                }
            }
        }

        if (enable_viz_diff){
            for (uint i = 0; i < pbs.size(); i++){
                std::string s = std::to_string(i);
                for (auto trkr : pbs.at(i).trackers) {
                    if (typeid(*trkr) == typeid(InsectTracker)){
                        InsectTracker * itrkr = static_cast<InsectTracker*>(trkr);
                        s = s + "->" + std::to_string(itrkr->id());
                    }
                }
                
                auto wblob = pbs.at(i).props->world_props;
                int code = 0;
                if (!wblob.valid && !pbs.at(i).ignored) {
                    if (!wblob.disparity_in_range)
                        code +=1;
                    if (!wblob.bkg_check_ok)
                        code +=2;
                    if (!wblob.radius_in_range)
                        code +=4;
                    if (pbs.at(i).props->ignores.size()>0)
                        code +=8;
                   if (wblob.takeoff_reject)
                        code +=16;
                    if (code>0)
                        s = s + "E" + std::to_string(code);
                }

                putText(diff_viz,s,pbs.at(i).pt()*pparams.imscalef,FONT_HERSHEY_SIMPLEX,0.5,pbs.at(i).color(),2);
                cv::circle(diff_viz,pbs.at(i).pt()*pparams.imscalef,3,pbs.at(i).color(),1);
            }
            for (auto trkr : replaytrackers()) {
                putText(diff_viz," r",trkr->image_item().pt()*pparams.imscalef,FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,180),2);
                cv::circle(diff_viz,trkr->image_item().pt()*pparams.imscalef,3,cv::Scalar(0,0,180),1);
            }
        }
    }
}

bool TrackerManager::tracker_active(ItemTracker * trkr, bool drone_is_active) {
    if (_mode == mode_idle)
        return false;
    else if (typeid(*trkr) == typeid(DroneTracker) && (_mode == mode_locate_drone || !drone_is_active)) {
        return false;
    } else if (typeid(*trkr) == typeid(InsectTracker) && (_mode == mode_locate_drone || _mode == mode_drone_only )) {
        return false;
    } else if (typeid(*trkr) == typeid(BlinkTracker) && (_mode != mode_locate_drone)) {
        return false;
    } else if (typeid(*trkr) == typeid(ReplayTracker)) {
        return false;
    }
    return true;
}

//Blob finder in the tracking ROI of the motion image. Looks for a limited number of
//maxima that are higher than a threshold, the area around the maximum
//is segmented from the background noise, and seen as a blob. It then
//tries if this blob can be further splitted if necessary.
void TrackerManager::update_max_change_points() {
    cv::Mat diff = _visdat->diffL_small;
    _blobs.clear();

    vizs_maxs.clear();
    uint blob_viz_cnt = 0;

    bool enable_take_off_split = false;
    bool enable_insect_drone_split = false;
    float drn_ins_split_thresh;

    if (_dtrkr->image_predict_item().valid) {
        if (_dtrkr->taking_off())
            enable_take_off_split = true;
        else  if ( insecttracker_best()->image_predict_item().valid)
            if (norm(_dtrkr->image_predict_item().pt() - insecttracker_best()->image_predict_item().pt()) < roi_radius){
                enable_insect_drone_split = true;
                drn_ins_split_thresh = insecttracker_best()->image_predict_item().pixel_max*0.2f;
            }
    }

    Mat bkg_frame = _visdat->motion_noise_map;
    for (int i = 0; i < max_points_per_frame; i++) {
        Point mint;
        Point maxt;
        double min, max;
        minMaxLoc(diff, &min, &max, &mint, &maxt);

        uint8_t bkg = bkg_frame.at<uint8_t>(maxt.y,maxt.x);

        int motion_thresh_tmp = motion_thresh;
        if (_mode == mode_locate_drone) {
            motion_thresh_tmp = motion_thresh + dparams.drone_blink_strength;
            bkg = 0; // motion noise calib is done during blink detection. To prevent interference do not use the bkg motion noise
        }

        bool thresh_res = max > bkg+motion_thresh_tmp;
        if (!thresh_res){ // specifcally for the insect tracker, check if there is an increased chance in this area
            for (uint j = 0; j < _trackers.size(); j++) {
                ItemTracker::ImagePredictItem ipi = _trackers.at(j)->image_predict_item();

                if (_trackers.at(j)->tracking() && typeid(*_trackers.at(j)) == typeid(InsectTracker)){
                    cv::Point2f d;
                    d.x = ipi.pt().x - maxt.x;
                    d.y = ipi.pt().y - maxt.y;
                    float dist = norm(d);
                    float chance = 1;
                    if (ipi.valid &&ipi.pixel_max < 1.5f * motion_thresh_tmp)
                        chance +=chance_multiplier_pixel_max;
                    if (dist < roi_radius){
                        chance += chance_multiplier_dist;
                    }
                    thresh_res = static_cast<uint8_t>(max) > bkg+(motion_thresh_tmp/chance);
                }
            }
        }

        if (thresh_res) {
            //find the COG:
            //get the Rect containing the max movement:
            Rect r2(maxt.x-roi_radius, maxt.y-roi_radius, roi_radius*2,roi_radius*2);
            if (r2.x < 0)
                r2.x = 0;
            else if (r2.x+r2.width >= diff.cols)
                r2.x -= (r2.x+r2.width+1) - diff.cols;
            if (r2.y < 0)
                r2.y = 0;
            else if (r2.y+r2.height >= diff.rows)
                r2.y -= (r2.y+r2.height+1) - diff.rows ;
            Mat roi(diff,r2); // so, this is the cut out around the max point

            // make a black mask, same size:
            Mat mask = Mat::zeros(roi.size(), roi.type());
            // with a white, filled circle in it:
            circle(mask, Point(roi_radius,roi_radius), roi_radius, 32765, -1);
            // combine roi & mask:
            Mat cropped = roi & mask;
            Scalar avg = mean(cropped);
            Scalar avg_bkg =mean(bkg_frame(r2));
            //blur, to filter out noise
            GaussianBlur(cropped,cropped,Size(5,5),0);

            //threshold to get only pixels that are heigher then the motion noise
            //mask = cropped > bkg_frame(r2)+1;
            if (enable_insect_drone_split)
                mask = cropped > drn_ins_split_thresh +  static_cast<float>(avg_bkg(0));
            else
                mask = cropped > (max-avg(0)) * 0.3;

            Moments mo = moments(mask,true);
            Point2f COG = Point2f(static_cast<float>(mo.m10) / static_cast<float>(mo.m00), static_cast<float>(mo.m01) / static_cast<float>(mo.m00));

            Mat viz;
            if (enable_viz_max_points){
                viz = create_row_image({roi,mask},CV_8UC1,4);
                cvtColor(viz,viz,CV_GRAY2BGR);
                if (enable_insect_drone_split)
                    putText(viz,"i-d",Point(0, viz.rows-13),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255,255,255));
            }
            bool viz_pushed = false;

            // relative it back to the _approx frame
            COG.x += r2.x;
            COG.y += r2.y;

            bool single_blob = true;
            bool COG_is_nan = false;

            if (enable_insect_drone_split || enable_take_off_split) {

                float dist_to_predict = norm(_dtrkr->image_predict_item().pt() - COG);
                if (dist_to_predict < 10) {

                    //check if the blob may be multiple blobs,
                    vector<vector<Point>> contours;
                    findContours(mask,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE); // If necessary, we could do this on the full resolution image...?
                    if (contours.size()==1 && enable_insect_drone_split) { // try another threshold value, sometimes we get lucky
                        drn_ins_split_thresh = insecttracker_best()->image_predict_item().pixel_max*0.3f;
                        mask = cropped > drn_ins_split_thresh + static_cast<float>(avg_bkg(0));
                        findContours(mask,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
                    }

                    if (contours.size()>1) {
                        //ok, definetely multiple blobs. Split them, and find the COG for each.
                        single_blob = false;
                        for (uint j = 0; j< contours.size();j++) {
                            Point2f COG2;
                            float radius;
                            if (contours.at(j).size() < 3) { // to prevnt COG nan
                                COG2 = contours.at(j).at(0);
                                radius = 1;
                            } else {
                                Moments mo2 = moments(contours.at(j),true);
                                COG2 = Point2f(static_cast<float>(mo2.m10) / static_cast<float>(mo2.m00), static_cast<float>(mo2.m01) / static_cast<float>(mo2.m00));
                                radius = mo2.m00;
                            }

                            if (COG2.x == COG2.x) {// if not nan
                                if (enable_viz_max_points){
                                    cv::Mat viz2 = viz.clone();
                                    circle(viz2,COG2*4,1,Scalar(0,0,255),1); //COG
                                    circle(viz2,COG2*4,roi_radius*4,Scalar(0,0,255),1);  // remove radius
                                    putText(viz2,to_string_with_precision(COG2.y*4,0),COG2*4,FONT_HERSHEY_SIMPLEX,0.4,Scalar(100,0,255));
                                    putText(viz2,std::to_string(blob_viz_cnt) + ", " + std::to_string(j) ,Point(0, 13),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255,255,0));
                                    blob_viz_cnt++;
                                    vizs_maxs.push_back(viz2);
                                    viz_pushed = true;
                                }
                                // relative COG back to the _approx frame, and save it:
                                COG2.x += r2.x;
                                COG2.y += r2.y;
                                uchar px_max = diff.at<uchar>(COG2);
                                _blobs.push_back(ItemTracker::BlobProps(COG2, radius, px_max, mask));

                                //remove this COG from the ROI:
                                circle(diff, COG2, 1, Scalar(0), roi_radius);
                            } else {
                                COG_is_nan = true;
                            }
                        }
                    }
                }
            }
            if (!viz_pushed &&  enable_viz_max_points) {
                putText(viz,std::to_string(blob_viz_cnt),Point(0, 13),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255,255,255));
                blob_viz_cnt++;
                vizs_maxs.push_back(viz);
            }
            if (single_blob) { // we could not split this blob, so we can use the original COG
                if (COG.x == COG.x) { // if not nan
                    _blobs.push_back(ItemTracker::BlobProps(COG, mo.m00,max, mask));
                    if (enable_viz_max_points) {
                        Point2f tmpCOG;
                        tmpCOG.x = COG.x - r2.x;
                        tmpCOG.y = COG.y - r2.y;
                        circle(viz,tmpCOG*4,1,Scalar(0,0,255),1);
                        circle(viz,tmpCOG*4,roi_radius*4,Scalar(0,0,255),1);  // remove radius
                        putText(viz,to_string_with_precision(tmpCOG.y*4,0),tmpCOG*4,FONT_HERSHEY_SIMPLEX,0.4,Scalar(255,0,0));
                    }
                    //remove this COG from the ROI:
                    circle(diff, COG, 1, Scalar(0), roi_radius);
                } else {
                    COG_is_nan = true;
                    if (enable_insect_drone_split){
                        //TODO: below is double code, streamline
                        for (uint j = 0; j < _trackers.size(); j++) {
                            ItemTracker::ImagePredictItem ipi = _trackers.at(j)->image_predict_item();
                            cv::Point2f d;
                            d.x = ipi.pt().x - maxt.x;
                            d.y = ipi.pt().y - maxt.y;
                            float dist = norm(d);
                            if (dist < roi_radius) {
                                _blobs.push_back(ItemTracker::BlobProps(maxt, 1,max, mask));
                                if (enable_viz_max_points) {
                                    Point2f tmpCOG;
                                    tmpCOG.x = maxt.x - r2.x;
                                    tmpCOG.y = maxt.y - r2.y;
                                    circle(viz,tmpCOG*4,1,Scalar(0,0,255),1);
                                    circle(viz,tmpCOG*4,roi_radius*4,Scalar(0,0,255),1);  // remove radius
                                    putText(viz,"maxt " + to_string_with_precision(tmpCOG.y*4,0),tmpCOG*4,FONT_HERSHEY_SIMPLEX,0.4,Scalar(255,0,0));
                                }
                                //remove this COG from the ROI:
                                circle(diff, maxt, 1, Scalar(0), roi_radius);
                                COG_is_nan = false;
                                break;
                            }
                        }
                    }
                }
            }
            if (COG_is_nan)  //remove the actual maximum from the ROI if the COG algorithm failed:
                circle(diff, maxt, 1, Scalar(0), roi_radius);

        } else {
            if  (static_cast<uint8_t>(max) <= bkg+(motion_thresh_tmp/chance_multiplier_total))
                break; // done searching for maxima, they are too small now
            else
                circle(diff, maxt, 1, Scalar(0), roi_radius);
        }
    }
}

std::vector<ReplayTracker *> TrackerManager::replaytrackers() {
    std::vector<ReplayTracker *> res;
    for (auto trkr : _trackers) {
        if (typeid(*trkr) == typeid(ReplayTracker)) {
            res.push_back(static_cast<ReplayTracker *>(trkr));
        }
    }
    return res;
}
std::vector<InsectTracker *> TrackerManager::insecttrackers() {
    std::vector<InsectTracker *> res;
    for (auto trkr : _trackers) {
        if (typeid(*trkr) == typeid(InsectTracker)) {
            res.push_back(static_cast<InsectTracker *>(trkr));
        }
    }
    //should do the same:
    //std::copy_if (_trackers.begin(), _trackers.end(), std::back_inserter(res), [](ItemTracker * trkr){return (typeid(*trkr) == typeid(InsectTracker));} );
    return res;
}
InsectTracker * TrackerManager::insecttracker_best(){
    float best_dist = 99999;
    InsectTracker * best_itrkr = default_itrkr;

    cv::Point3f current_drone_pos = _dtrkr->Last_track_data().pos();
    if (_dtrkr->Last_track_data().pos_valid){
        current_drone_pos = _dtrkr->drone_startup_location();
    }

    for (auto trkr : insecttrackers()) {
        if (trkr->tracking() ){
            float dist = normf(current_drone_pos- trkr->Last_track_data().pos());
            if (best_dist > dist){
                dist = best_dist;
                best_itrkr = trkr;
            }
        }
    }
    for (auto trkr : replaytrackers()) {
        if (trkr->tracking() ){
            float dist = normf(current_drone_pos- trkr->Last_track_data().pos());
            if (best_dist > dist){
                dist = best_dist;
                best_itrkr = trkr;
            }
        }
    }

    return best_itrkr;
}
void TrackerManager::deserialize_settings() {
    std::cout << "Reading settings from: " << settings_file << std::endl;
    TrackerManagerParameters params;
    if (file_exist(settings_file)) {
        std::ifstream infile(settings_file);
        std::string xmlData((std::istreambuf_iterator<char>(infile)),
                            std::istreambuf_iterator<char>());

        if (!xmls::Serializable::fromXML(xmlData, &params))
        { // Deserialization not successful
            throw my_exit("Cannot read: " + settings_file);
        }
        TrackerManagerParameters tmp;
        auto v1 = params.getVersion();
        auto v2 = tmp.getVersion();
        if (v1 != v2) {
            throw my_exit("XML version difference detected from " + settings_file);
        }
    } else {
        throw my_exit("File not found: " + settings_file);
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

void TrackerManager::close () {
    if (initialized){
        for (uint i=0; i < _trackers.size(); i++)
            _trackers.at(i)->close();
        //        serialize_settings();
        initialized = false;
    }
}
