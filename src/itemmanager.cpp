#include "itemmanager.h"
using namespace cv;
using namespace std;

void ItemManager::init(std::ofstream *logger,VisionData *visdat){
    _visdat = visdat;
    _logger = logger;
#ifdef HASSCREEN
    enable_viz_max_points = true;
    enable_viz_diff = true;
#endif
    _settingsFile = "../itemmanager_settings.dat";
    if (checkFileExist(_settingsFile)) {
        std::ifstream is(_settingsFile, std::ios::binary);
        cereal::BinaryInputArchive archive( is );
        try {
            archive(settings);
        }catch (cereal::Exception e) {
            std::stringstream serr;
            serr << "cannot read itemmanager settings file: " << e.what() << ". Maybe delete the file: " << _settingsFile;
            throw my_exit(serr.str());
        }
        item_manager_settings tmp;
        if (tmp.version-settings.version > 0.001f){
            std::stringstream serr;
            serr << "itemmanager settings version too low! Maybe delete the file: " << _settingsFile;
            throw my_exit(serr.str());
        }
    }
    _dtrkr = new DroneTracker();
    _itrkr = new InsectTracker();
    _dtrkr->init(_logger,_visdat);
    _itrkr->init(_logger,_visdat);
    _trackers.push_back(_dtrkr);
    _trackers.push_back(_itrkr);

    initialized = true;
}

void ItemManager::update(double time,LogReader::Log_Entry log_entry, bool drone_is_active){
    if (enable_viz_diff)
        cv::cvtColor(_visdat->diffL*10,diff_viz,CV_GRAY2BGR);
    //find max points in the whole image
    //for each tracker,
    //  check if it is not overlapping with a static ignore zone (maybe make this a negative scoring factor), if OK:
    //      calculate a match based on the distance to its prediction and the last known size (todo: smooth size)
    //select the max point that has the best score

    //
    //check if there are conflicts, resolve them somehow

    //for each tracker, update the predicted location

    if (_mode != mode_idle){
        update_max_change_points();

        update_static_ignores();

        match_image_points_to_trackers(drone_is_active);

        //check if there are conflicts:
        //        for (uint i=0; i<_trackers.size();i++){
        //            for (uint j=i+1; j<_trackers.size();j++){
        //                if (_trackers.at(i)->image_track_item().keypoint_id ==_trackers.at(j)->image_track_item().keypoint_id){
        //                    //trouble.
        //                    //todo: is it possible to split this blob?
        //                }
        //            }
        //        }

        update_trackers(time,log_entry, drone_is_active);
    }
}

void ItemManager::update_trackers(double time,LogReader::Log_Entry log_entry, bool drone_is_active) {
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
            if (!_trackers.at(i)->world_track_item().valid) {
                putText(diff_viz,"E",_trackers.at(i)->image_track_item().pt()*IMSCALEF,FONT_HERSHEY_SIMPLEX,0.9,tracker_color(_trackers.at(i)));
            }
        } else if (typeid(*_trackers.at(i)) == typeid(InsectTracker)) {
            InsectTracker * itrkr = static_cast<InsectTracker * >(_trackers.at(i));
            switch (_mode){
            case mode_idle:{
                itrkr->append_log(); // write dummy data
                break;
            } case mode_locate_drone:{
                itrkr->append_log(); // write dummy data
                break;
            } case mode_wait_for_insect:{
                itrkr->track(time);
                break;
            } case mode_hunt:{
                itrkr->track(time);
                break;
            } case mode_hunt_replay_moth:{
                itrkr->update_from_log(log_entry,_visdat->frame_id);
                break;
            } case mode_drone_only:{
                itrkr->append_log(); // write dummy data
                break;
            }
            }
            if (!_trackers.at(i)->world_track_item().valid) {
                putText(diff_viz,"E",_trackers.at(i)->image_track_item().pt()*IMSCALEF,FONT_HERSHEY_SIMPLEX,0.9,tracker_color(_trackers.at(i)));
            }
        } else if (typeid(*_trackers.at(i)) == typeid(BlinkTracker)) {
            if (_mode == mode_locate_drone) {
                BlinkTracker * btrkr = static_cast<BlinkTracker * >(_trackers.at(i));
                btrkr->track(time);
                if (btrkr->state() == BlinkTracker::bds_found){
                    _mode = mode_wait_for_insect;
                    _dtrkr->set_drone_landing_location(btrkr->image_track_item().pt(),btrkr->world_track_item().pt);
                    _trackers.erase(_trackers.begin() + i);
                    delete btrkr;
                }
            } else {
                BlinkTracker * btrkr = static_cast<BlinkTracker * >(_trackers.at(i));
                _trackers.erase(_trackers.begin() + i);
                delete btrkr;
            }
        }
    }
}

//for each tracker collect the static ignores from each other tracker
void ItemManager::update_static_ignores() {
    for (uint i=0; i<_trackers.size();i++){
        std::vector<ItemTracker::StaticIgnorePoint> ignores;
        for (uint j=0; j<_trackers.size();j++){
            if (j!=i){
                for (uint k = 0; k < _trackers.at(j)->static_ignores_points_for_other_trkrs.size();k++){
                    ignores.push_back(_trackers.at(j)->static_ignores_points_for_other_trkrs.at(k));
                }
            }
        }
        _trackers.at(i)->static_ignores_points_for_me = ignores;
    }
}

bool ItemManager::check_ignore_points(processed_max_point pmp, ItemTracker * trkr) {
    bool in_ignore_zone = false;
    cv::Point2f p_kp = pmp.pt;
    for (uint k=0; k<trkr->static_ignores_points_for_me.size();k++){
        cv::Point2f p_ignore = trkr->static_ignores_points_for_me.at(k).p;
        float dist_ignore = sqrtf(powf(p_ignore.x-p_kp.x,2)+powf(p_ignore.y-p_kp.y,2));
        if (dist_ignore < settings.static_ignores_dist_thresh){
            pmp.color = cv::Scalar(0,128,0);
            trkr->static_ignores_points_for_me.at(k).was_used = true;
            in_ignore_zone = true;
        }
    }

    return in_ignore_zone;
}

void ItemManager::match_image_points_to_trackers(bool drone_is_active) {

    //set all trackers to invalid, so that it is noticed when tracking is lost.
    ItemTracker::ImageTrackItem invalid;
    for (uint i=0; i<_trackers.size();i++)
        _trackers.at(i)->image_track_item(invalid);

    if (_kps.size()>0) {
        //init keypoints list
        std::vector<processed_max_point> pmps;
        for (uint i=0; i < _kps.size(); i++)
            pmps.push_back(processed_max_point(_kps.at(i)));

        //first check if there are trackers, that were already tracking something, which prediction matches a new keypoint
        for (uint i=0; i<_trackers.size();i++){
            ItemTracker * trkr = _trackers.at(i);
            float best_score = -1;
            float best_dist;
            int best_score_j;

            if (tracker_active(trkr,drone_is_active) && trkr->tracking()) {
                ItemTracker::ImagePredictItem ipi = trkr->predicted_image_path.back();
                cv::Point2f p_predict(ipi.x,ipi.y);

                for (uint j=0; j < pmps.size(); j++) {
                    cv::Point2f p_kp = pmps.at(j).pt;
                    float score =0;
                    float dist = 99;
                    //check against static ignore points
                    bool in_ignore_zone = check_ignore_points(pmps.at(j),trkr);

                    if (!in_ignore_zone) {
                        dist = sqrtf(powf(p_predict.x-p_kp.x,2)+powf(p_predict.y-p_kp.y,2));
                        float im_size_diff =0;
                        if (trkr->path.size() > 0)
                            im_size_diff = fabs(trkr->path.back().size_in_image() - pmps.at(j).size) / pmps.at(j).size; // TODO: use prediction for size as well
                        score = 1.f / (dist + 5.f*im_size_diff); // TODO: ipi.certainty  not working properly

                        if (score > best_score){
                            best_score = score;
                            best_score_j = j;
                            best_dist = dist;
                        }
                    }
                }
                if (best_score>=0){
                    ItemTracker::ImageTrackItem iti(_kps.at(best_score_j),_visdat->frame_id,best_dist,best_score,best_score_j);
                    trkr->image_track_item(iti);
                    pmps.at(best_score_j).tracked = true;
                    pmps.at(best_score_j).color = tracker_color(trkr);
                }
            }

        }

        //see if there are trackers that are not tracking yet and if there are untracked points left, which can be bound together.
        for (uint i=0; i<_trackers.size();i++){
            ItemTracker * trkr = _trackers.at(i);
            if (!trkr->tracking()) {
                if (tracker_active(trkr, drone_is_active)) {
                    for (uint j=0; j < pmps.size(); j++) {
                        if (!pmps.at(j).tracked){

                            //check against static ignore points TODO: function
                            bool in_ignore_zone = check_ignore_points(pmps.at(j),trkr);

                            //TODO: in theory this would be a very nice time to check the world situation as well..
                            if (!in_ignore_zone) {
                                // the tracker has lost the item, or is still initializing.
                                //there is no valid prediction, the score is therefor as low as possible
                                ItemTracker::ImageTrackItem iti(_kps.at(j),_visdat->frame_id,0,0,j);
                                trkr->image_track_item(iti);
                                pmps.at(j).tracked = true;
                                pmps.at(j).color = tracker_color(trkr);
                                break;
                            }
                        }
                    }
                }
            }
        }

        //see if there are static ignore points that are detected. If so set was_used flag
        for (uint i=0; i<_trackers.size();i++){
            ItemTracker * trkr = _trackers.at(i);
            for (uint j=0; j < pmps.size(); j++) {
                cv::Point2f p_kp = pmps.at(j).pt;
                for (uint k=0; k<trkr->static_ignores_points_for_other_trkrs.size();k++){
                    cv::Point2f p_ignore = trkr->static_ignores_points_for_other_trkrs.at(k).p;
                    float dist_ignore = sqrtf(powf(p_ignore.x-p_kp.x,2)+powf(p_ignore.y-p_kp.y,2));
                    if (dist_ignore < settings.static_ignores_dist_thresh){
                        trkr->static_ignores_points_for_other_trkrs.at(k).was_used = true;
                    }
                }
            }
        }

        //see if there are still keypoints left untracked, create new trackers for them
        for (uint i=0; i < _kps.size(); i++){
            if (!pmps.at(i).tracked && _trackers.size() < 3) { // if so, start tracking it!
                if (_mode == mode_locate_drone){
                    BlinkTracker  * bt;
                    bt = new BlinkTracker();
                    bt->init(_logger,_visdat);
                    bt->image_track_item(ItemTracker::ImageTrackItem (_kps.at(i),_visdat->frame_id,0,0,i));
                    _trackers.push_back( bt);
                    pmps.at(i).tracked = true;
                    pmps.at(i).color = tracker_color(bt);
                    break; // only add one tracker per frame
                }
                break;
            } // todo: add support for multiple insect trackers
        }

        if (enable_viz_diff){
            for (uint i = 0; i < pmps.size(); i++){
                putText(diff_viz,std::to_string(i),pmps.at(i).pt*IMSCALEF,FONT_HERSHEY_SIMPLEX,0.5,pmps.at(i).color);
                cv::circle(diff_viz,pmps.at(i).pt*IMSCALEF,3,pmps.at(i).color,1);
            }
        }
    }
}



bool ItemManager::tracker_active(ItemTracker * trkr, bool drone_is_active) {
    if (_mode == mode_idle)
        return false;
    else if (typeid(*trkr) == typeid(DroneTracker) && (_mode == mode_locate_drone || !drone_is_active)) {
        return false;
    } else if (typeid(*trkr) == typeid(InsectTracker) && (_mode == mode_locate_drone || _mode == mode_drone_only )) {
        return false;
    } else if (typeid(*trkr) == typeid(BlinkTracker) && (_mode != mode_locate_drone)) {
        return false;
    }
    return true;
}

cv::Scalar ItemManager::tracker_color(ItemTracker * trkr) {
    if (typeid(*trkr) == typeid(DroneTracker))
        return cv::Scalar(0,255,0);
    else if (typeid(*trkr) == typeid(InsectTracker))
        return cv::Scalar(0,0,255);
    else if (typeid(*trkr) == typeid(BlinkTracker))
        return cv::Scalar(30,30,200);
    return cv::Scalar(0,0,0);
}

//Blob finder in the tracking ROI of the motion image. Looks for a limited number of
//maxima that are higher than a threshold, the area around the maximum
//is segmented from the background noise, and seen as a blob. It then
//tries if this blob can be further splitted if necessary.
void ItemManager::update_max_change_points() {
    cv::Mat diff = _visdat->diffL_small;
    _kps.clear();

    std::vector<Mat> vizs_maxs;

    for (int i = 0; i < settings.max_points_per_frame; i++) {
        Point mint;
        Point maxt;
        double min, max;
        minMaxLoc(diff, &min, &max, &mint, &maxt);

        Mat bkg_frame_cutout = _visdat->max_uncertainty_map;
        uint8_t bkg = bkg_frame_cutout.at<uint8_t>(maxt.y,maxt.x);
        if (!_enable_motion_background_check) // TODO: can be replaced by _mode
            bkg = 0;

        int motion_thresh = settings.motion_thresh;
        if (_mode == mode_locate_drone)
            motion_thresh = settings.motion_thresh_blink_detect;

        if (max > bkg+motion_thresh) {
            //find the COG:
            //get the Rect containing the max movement:
            Rect r2(maxt.x-settings.radius, maxt.y-settings.radius, settings.radius*2,settings.radius*2);
            if (r2.x < 0)
                r2.x = 0;
            else if (r2.x+r2.width >= diff.cols)
                r2.x -= (r2.x+r2.width+1) - diff.cols;
            if (r2.y < 0)
                r2.y = 0;
            else if (r2.y+r2.height >= diff.rows)
                r2.y -= (r2.y+r2.height+1) - diff.rows ;
            Mat roi(diff,r2); // so, this is the cut out around the max point, in the cut out of _approx roi

            // make a black mask, same size:
            Mat mask = Mat::zeros(roi.size(), roi.type());
            // with a white, filled circle in it:
            circle(mask, Point(settings.radius,settings.radius), settings.radius, 32765, -1);
            // combine roi & mask:
            Mat cropped = roi & mask;
            Scalar avg = mean(cropped);
            GaussianBlur(cropped,cropped,Size(5,5),0);
            mask = cropped > (max-avg(0)) * 0.3; // TODO: factor 0,3 seems to work better than 0,5, but what makes sense here?
            cropped = mask;

            Moments mo = moments(cropped,true);
            Point2f COG = Point2f(static_cast<float>(mo.m10) / static_cast<float>(mo.m00), static_cast<float>(mo.m01) / static_cast<float>(mo.m00));

            Mat viz;
            if (enable_viz_max_points){
                viz = createRowImage({roi,mask},CV_8UC1,4);
                cvtColor(viz,viz,CV_GRAY2BGR);
                putText(viz,std::to_string(i),Point(0, 13),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255,255,0));
                vizs_maxs.push_back(viz);
            }

            // relative it back to the _approx frame
            COG.x += r2.x;
            COG.y += r2.y;

            //check if the blob may be multiple blobs, first check distance between COG and max:
            float dist = pow(COG.x-maxt.x,2) + pow(COG.y-maxt.y,2);
            bool single_blob = true;
            bool COG_is_nan = false;
            if (dist > 4) {//todo: instead of just check the distance, we could also check the pixel value of the COG, if it is black it is probably multiple blobs
                //the distance between the COG and the max is quite large, now check if there are multiple contours:
                vector<vector<Point>> contours;
                findContours(cropped,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
                if (contours.size()>1) {
                    //ok, definetely multiple blobs. Split them, and find the COG for each.
                    single_blob = false;
                    for (uint j = 0; j< contours.size();j++) {
                        Moments mo2 = moments(contours.at(j),true);
                        Point2f COG2 = Point2f(static_cast<float>(mo2.m10) / static_cast<float>(mo2.m00), static_cast<float>(mo2.m01) / static_cast<float>(mo2.m00));

                        if (COG2.x == COG2.x) {// if not nan
                            if (enable_viz_max_points){
                                circle(viz,COG2*4,1,Scalar(0,0,255),1);
                                putText(viz,to_string_with_precision(COG2.y*4,0),COG2*4,FONT_HERSHEY_SIMPLEX,0.4,Scalar(100,0,255));
                            }
                            // relative COG back to the _approx frame, and save it:
                            COG2.x += r2.x;
                            COG2.y += r2.y;
                            _kps.push_back(KeyPoint(COG2, mo2.m00));

                            //remove this COG from the ROI:
                            circle(diff, COG2, 1, Scalar(0), settings.radius);

                        } else {
                            COG_is_nan = true;
                        }
                    }
                }
            }
            if (single_blob) { // we could not split this blob, so we can use the original COG
                if (COG.x == COG.x) { // if not nan
                    _kps.push_back(KeyPoint(COG, mo.m00));
                    if (enable_viz_max_points) {
                        Point2f tmpCOG;
                        tmpCOG.x = COG.x - r2.x;
                        tmpCOG.y = COG.y - r2.y;
                        circle(viz,tmpCOG*4,1,Scalar(0,0,255),1);
                    }
                    //remove this COG from the ROI:
                    circle(diff, COG, 1, Scalar(0), settings.radius);
                } else {
                    COG_is_nan = true;
                }
            }
            if (COG_is_nan)  //remove the actual maximum from the ROI if the COG algorithm failed:
                circle(diff, maxt, 1, Scalar(0), settings.radius);

        } else {
            break; // done searching for maxima, they are too small now
        }
    }


    if (enable_viz_max_points && vizs_maxs.size()>0)
        viz_max_points = createColumnImage(vizs_maxs,CV_8UC3,1);

}

void ItemManager::close () {
    if (initialized){
        for (uint i=0; i < _trackers.size(); i++)
            _trackers.at(i)->close();
        std::cout << "Closing item manager" << std::endl;
        std::ofstream os(_settingsFile, std::ios::binary);
        cereal::BinaryOutputArchive archive( os );
        archive( settings );
        initialized = false;
    }
}
