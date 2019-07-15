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

    //we have a bit of a situation with the order of initializing the trackers, as this MUST be in the same order as how they are called
    //in the future this is prolly not sustainable, as we have multiple insect trackers etc
    //ATM the blinktracker is therefor not logged, as there can be multiple and they disappear after they are done
    //Anyway, the track() functions are called in a reverse for loop (must be reversed because items are erased from the list),
    //therefor the init functions must be called in the reverser order as adding them to the _trackers list...
    //...but since we are 'pushing' the items in, this in the end must be in the normal order. Or not. I guess it's a 50% chance thing.
    //TODO: make issue to remove this complexity.
    _dtrkr = new DroneTracker();
    _itrkr = new InsectTracker();
    _dtrkr->init(_logger,_visdat);
    _itrkr->init(_logger,_visdat);
    _trackers.push_back(_itrkr);
    _trackers.push_back(_dtrkr);

    initialized = true;
}

void ItemManager::update(double time,LogReader::Log_Entry log_entry, bool drone_is_active){
    if (enable_viz_diff) {
        cv::cvtColor(_visdat->diffL*10,diff_viz,CV_GRAY2BGR);
        putText(diff_viz,"Drone",cv::Point(3,diff_viz.rows-12),FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(0,255,0));
        putText(diff_viz,"Insect",cv::Point(3,diff_viz.rows-24),FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(0,0,255));
        putText(diff_viz,"Blink",cv::Point(3,diff_viz.rows-36),FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(255,0,255));
        putText(diff_viz,"Ignored",cv::Point(3,diff_viz.rows-48),FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(0,128,0));
        putText(diff_viz,"Untracked",cv::Point(3,diff_viz.rows-60),FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(255,255,55));
        putText(diff_viz,"Multitracked",cv::Point(3,diff_viz.rows-72),FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(200,255,250));
    }

    if (_mode != mode_idle){
        update_max_change_points();

        update_static_ignores();

        match_blobs_to_trackers(drone_is_active);

    }

    update_trackers(time,log_entry, drone_is_active);

    if (enable_viz_max_points && vizs_maxs.size()>0)
        viz_max_points = createColumnImage(vizs_maxs,CV_8UC3,1);
    else if(enable_viz_max_points)
        viz_max_points = cv::Mat::zeros(5,100,CV_8UC3);

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
            if (!_trackers.at(i)->world_item().valid && _trackers.at(i)->image_item().valid) {
                putText(diff_viz,"W",_trackers.at(i)->image_item().pt()*IMSCALEF,FONT_HERSHEY_SIMPLEX,0.9,cv::Scalar(0,255,0));
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
            if (!_trackers.at(i)->world_item().valid && _trackers.at(i)->image_item().valid) {
                putText(diff_viz,"W",_trackers.at(i)->image_item().pt()*IMSCALEF,FONT_HERSHEY_SIMPLEX,0.9,cv::Scalar(0,0,255));
            }
        } else if (typeid(*_trackers.at(i)) == typeid(BlinkTracker)) {
            BlinkTracker * btrkr = static_cast<BlinkTracker * >(_trackers.at(i));
            btrkr->track(time);
            if (_mode == mode_locate_drone) {
                if (btrkr->state() == BlinkTracker::bds_found){
                    _dtrkr->set_drone_landing_location(btrkr->image_item().pt(),btrkr->smoothed_size_image(),btrkr->world_item().pt);
                    _mode = mode_wait_for_insect;
                }
            } else if (btrkr->ignores_for_other_trkrs.size() == 0){
                _trackers.erase(_trackers.begin() + i);
                delete btrkr;
            }
        }
    }
}

//for each tracker collect the static ignores from each other tracker
void ItemManager::update_static_ignores() {
    bool landingspot_ignore_found = false;
    ItemTracker::IgnoreBlob landingspot;
    for (uint i=0; i<_trackers.size();i++){
        std::vector<ItemTracker::IgnoreBlob> ignores;
        for (uint j=0; j<_trackers.size();j++){
            if (j!=i){
                for (uint k = 0; k < _trackers.at(j)->ignores_for_other_trkrs.size();k++){
                    ignores.push_back(_trackers.at(j)->ignores_for_other_trkrs.at(k));
                    if (_trackers.at(j)->ignores_for_other_trkrs.at(k).ignore_type == ItemTracker::IgnoreBlob::landing_spot){
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

void ItemManager::match_blobs_to_trackers(bool drone_is_active) {

    //set all trackers to invalid so we can use this as a flag to notice when tracking is lost.
    for (uint i=0; i<_trackers.size();i++)
        _trackers.at(i)->image_item_invalidize();

    if (_blobs.size()>0) {
        //init keypoints list
        std::vector<processed_blobs> pbs;
        for (uint i=0; i < _blobs.size(); i++)
            pbs.push_back(processed_blobs(_blobs.at(i)));

        //first check if there are trackers, that were already tracking something, which prediction matches a new keypoint
        for (uint i=0; i<_trackers.size();i++){
            ItemTracker * trkr = _trackers.at(i);
            float best_score = -1;
            int best_score_j;

            if (tracker_active(trkr,drone_is_active) && trkr->tracking()) {
                for (uint j=0; j < pbs.size(); j++) {
                    float score =0;
                    //check against static ignore points
                    ItemTracker::BlobProps tmp = pbs.at(j).props();
                    bool in_ignore_zone = trkr->check_ignore_blobs(&tmp);
                    if (in_ignore_zone)
                        pbs.at(j).ignored = true;
                    if (!in_ignore_zone) {
                        score  = trkr->score(_blobs.at(j));
                        if (score > best_score){
                            best_score = score;
                            best_score_j = j;
                        }
                        if (enable_viz_max_points) {
                            cv::Mat viz = vizs_maxs.at(j);
                            cv::Point2i pt(viz.cols-50,viz.rows - 14*(i+1));
                            putText(viz,to_string_with_precision(score*1000,1),pt,FONT_HERSHEY_SIMPLEX,0.5,tracker_color(trkr));
                        }
                    } else if (enable_viz_max_points){
                        cv::Mat viz = vizs_maxs.at(j);
                        cv::Point2i pt(viz.cols-50,viz.rows - 14*(i+1));
                        putText(viz,"Ign.",pt,FONT_HERSHEY_SIMPLEX,0.5,tracker_color(trkr));
                    }
                }
                if (best_score >= trkr->score_threshold() ) {
                    ItemTracker::ImageItem iti(_blobs.at(best_score_j),_visdat->frame_id,best_score,best_score_j);
                    trkr->image_item(iti);
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
                            ItemTracker::BlobProps tmp = pbs.at(j).props();
                            bool in_ignore_zone = trkr->check_ignore_blobs(&tmp);
                            if (in_ignore_zone)
                                pbs.at(j).ignored = true;

                            //TODO: in theory this would be a very nice time to check the world situation as well..
                            if (!in_ignore_zone) {
                                //The tracker has lost the item, or is still initializing.
                                //there is no valid prediction, the score is therefor as low as possible
                                ItemTracker::ImageItem iti(_blobs.at(j),_visdat->frame_id,0,j);
                                trkr->image_item(iti);
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
        for (uint i=0; i<_trackers.size();i++){
            ItemTracker * trkr = _trackers.at(i);
            for (uint j=0; j < pbs.size(); j++) {
                cv::Point2f p_kp = pbs.at(j).pt;
                for (uint k=0; k<trkr->ignores_for_other_trkrs.size();k++){
                    cv::Point2f p_ignore = trkr->ignores_for_other_trkrs.at(k).p;
                    float dist_ignore = sqrtf(powf(p_ignore.x-p_kp.x,2)+powf(p_ignore.y-p_kp.y,2));
                    if (dist_ignore < settings.static_ignores_dist_thresh){
                        trkr->ignores_for_other_trkrs.at(k).was_used = true;
                    }
                }
            }
        }

        //see if there are still keypoints left untracked, create new trackers for them
        for (uint i=0; i < _blobs.size(); i++){
            if (!pbs.at(i).tracked() && _trackers.size() < 3) { // if so, start tracking it!
                if (_mode == mode_locate_drone){
                    BlinkTracker  * bt;
                    bt = new BlinkTracker();
                    bt->init(_visdat);
                    bt->image_item(ItemTracker::ImageItem (_blobs.at(i),_visdat->frame_id,100,i));
                    _trackers.push_back( bt);
                    pbs.at(i).trackers.push_back(bt);
                    break; // only add one tracker per frame
                }
            } // todo: add support for multiple insect trackers
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
                ItemTracker * t1 = pbs.at(i).trackers.at(0);
                ItemTracker * t2 = pbs.at(i).trackers.at(1); //todo: make this general for more then 2
                bool drn_trkr_fnd = false;
                bool irn_trkr_fnd = false;

                if (typeid(*t1) == typeid(DroneTracker)) {
                    dtrkr = static_cast<DroneTracker*>(t1);
                    drn_trkr_fnd = true;
                } else if (typeid(*t1) == typeid(InsectTracker)) {
                    itrkr = static_cast<InsectTracker*>(t1);
                    irn_trkr_fnd = true;
                }
                if (typeid(*t2) == typeid(DroneTracker)) {
                    dtrkr = static_cast<DroneTracker*>(t2);
                    drn_trkr_fnd = true;
                } else if (typeid(*t2) == typeid(InsectTracker)) {
                    itrkr = static_cast<InsectTracker*>(t2);
                    irn_trkr_fnd = true;
                }

                if (drn_trkr_fnd && irn_trkr_fnd){
                    //so, there should be another blob close to the currently one used.
                    //if there is, the small one is prolly the moth...
                    bool conflict_resolved = false;
                    for (uint j = 0; j < pbs.size();j++){
                        if (i!=j && !pbs.at(j).tracked()){
                            float dist = cv::norm(pbs.at(i).pt-pbs.at(j).pt);
                            if (dist < 2.f* (pbs.at(i).size + pbs.at(j).size)) {
                                if(pbs.at(i).size > pbs.at(j).size) {
                                    dtrkr->image_item(ItemTracker::ImageItem (_blobs.at(i),_visdat->frame_id,dtrkr->score(_blobs.at(i)),i));
                                    itrkr->image_item(ItemTracker::ImageItem (_blobs.at(j),_visdat->frame_id,itrkr->score(_blobs.at(j)),j));
                                    pbs.at(i).trackers.clear();
                                    pbs.at(j).trackers.clear();
                                    pbs.at(i).trackers.push_back(dtrkr);
                                    pbs.at(j).trackers.push_back(itrkr);
                                }else {
                                    itrkr->image_item(ItemTracker::ImageItem (_blobs.at(i),_visdat->frame_id,itrkr->score(_blobs.at(i)),i));
                                    dtrkr->image_item(ItemTracker::ImageItem (_blobs.at(j),_visdat->frame_id,dtrkr->score(_blobs.at(j)),j));
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
                        _itrkr->make_image_item_size_invald();
                        //TODO: do something sensible in cases 1,2,4:
                        // hmm, apparantely there is no blob close by, so now there are two possibilities:
                        //1. The insect is lost (e.g. too far, too small, into the flowers)
                        //2. The drone is lost (e.g. crashed)
                        //3. The insect and drone are too close to eachother to distinguish
                        //4. We have a kill :)
                    }
                }
            }
        }

        if (enable_viz_diff){
            for (uint i = 0; i < pbs.size(); i++){
                putText(diff_viz,std::to_string(i),pbs.at(i).pt*IMSCALEF,FONT_HERSHEY_SIMPLEX,0.5,pbs.at(i).color(),2);
                cv::circle(diff_viz,pbs.at(i).pt*IMSCALEF,3,pbs.at(i).color(),1);
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

//Blob finder in the tracking ROI of the motion image. Looks for a limited number of
//maxima that are higher than a threshold, the area around the maximum
//is segmented from the background noise, and seen as a blob. It then
//tries if this blob can be further splitted if necessary.
void ItemManager::update_max_change_points() {
    cv::Mat diff = _visdat->diffL_small;
    _blobs.clear();

    vizs_maxs.clear();
    uint blob_viz_cnt = 0;

    for (int i = 0; i < settings.max_points_per_frame; i++) {
        Point mint;
        Point maxt;
        double min, max;
        minMaxLoc(diff, &min, &max, &mint, &maxt);

        Mat bkg_frame_cutout = _visdat->max_uncertainty_map;
        uint8_t bkg = bkg_frame_cutout.at<uint8_t>(maxt.y,maxt.x);

        int motion_thresh = settings.motion_thresh;
        if (_mode == mode_locate_drone) {
            motion_thresh = settings.motion_thresh_blink_detect;
            bkg = 0;
        }
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
            }
            bool viz_pushed = false;

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
                                cv::Mat viz2 = viz.clone();
                                circle(viz2,COG2*4,1,Scalar(0,0,255),1);
                                putText(viz2,to_string_with_precision(COG2.y*4,0),COG2*4,FONT_HERSHEY_SIMPLEX,0.4,Scalar(100,0,255));
                                putText(viz2,std::to_string(blob_viz_cnt),Point(0, 13),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255,255,255));
                                blob_viz_cnt++;
                                vizs_maxs.push_back(viz2);
                                viz_pushed = true;
                            }
                            // relative COG back to the _approx frame, and save it:
                            COG2.x += r2.x;
                            COG2.y += r2.y;
                            _blobs.push_back(ItemTracker::BlobProps(COG2, mo2.m00, max));

                            //remove this COG from the ROI:
                            circle(diff, COG2, 1, Scalar(0), settings.radius);

                        } else {
                            COG_is_nan = true;
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
                    _blobs.push_back(ItemTracker::BlobProps(COG, mo.m00,max));
                    if (enable_viz_max_points) {
                        Point2f tmpCOG;
                        tmpCOG.x = COG.x - r2.x;
                        tmpCOG.y = COG.y - r2.y;
                        circle(viz,tmpCOG*4,1,Scalar(0,0,255),1);
                        putText(viz,to_string_with_precision(tmpCOG.y*4,0),tmpCOG*4,FONT_HERSHEY_SIMPLEX,0.4,Scalar(100,0,255));
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
