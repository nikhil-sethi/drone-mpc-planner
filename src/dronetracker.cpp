#include "dronetracker.h"

#ifdef HASSCREEN
//#define VIZ
#endif

bool DroneTracker::init(std::ofstream *logger, VisionData *visdat, bool fromfile, std::string bag_dir) {
    ItemTracker::init(logger,visdat,"drone");

    if (fromfile)
        deserialize_calib(bag_dir + '/' + calib_fn );
    calib_fn = "./logging/" + calib_fn ;

    return false;
}
void DroneTracker::init_settings() {

    settings.min_disparity=1;
    settings.max_disparity=43;
    settings.motion_thresh = 30;

    settings.radius = 15;

    settings.roi_min_size = 150;
    settings.roi_max_grow = 50;
    settings.roi_grow_speed = 64;

    settings.background_subtract_zone_factor = 97;
}

void DroneTracker::track(float time, std::vector<track_item> ignore, bool drone_is_active) {
    std::vector<cv::Point2f> additional_ignores;
    current_time = time;
#ifdef VIZ
    cv::cvtColor(_visdat->diffL*10,diff_viz,CV_GRAY2BGR);
#endif

    switch (_drone_tracking_status) {
    case dts_init: {
        append_log(); // no tracking needed in this stage
        break;
    } case dts_blinking:
        roi_size_cnt = 0; // don't grow roi in this stage
        start_take_off_time = time;
        ignore.clear(); // insect tracker may pick up something...
        switch (_blinking_drone_status) {
        case bds_start: {
            _enable_roi = false;
            _enable_depth_background_check = false;
            _enable_motion_background_check = false;
            pathL.clear();
            predicted_pathL.clear();
            foundL = false; // todo: is this necessary?
            //todo: also disable background depth map?
            _blinking_drone_status = bds_reset_bkg;
            _visdat->reset_motion_integration();
            ItemTracker::append_log(); // write a dummy entry
            break;
        } case bds_reset_bkg: {
            _blinking_drone_status = bds_searching; // -> wait 1 frame
            ItemTracker::append_log(); // write a dummy entry
            break;
        } case bds_searching: {
#ifdef MANUAL_DRONE_LOCATE
            append_log();
            _blinking_drone_status = bds_found;
            break;
#endif
            _enable_roi = false;
            ItemTracker::track(time,ignore,additional_ignores);
            if (n_frames_lost == 0) {
                _blinking_drone_status = bds_1_blink_off;
                blink_time_start = time;
            }
            break;
        } case bds_1_blink_off: {
            _enable_roi = true;
            roi_size_cnt = 0;
            blink_location = find_result.best_image_locationL;
            ItemTracker::track(time,ignore,additional_ignores);
            _blinking_drone_status = detect_blink(time, n_frames_tracking == 0);
            break;
        } case bds_1_blink_on: {
            roi_size_cnt = 0;
            ItemTracker::track(time,ignore,additional_ignores);
            _blinking_drone_status = detect_blink(time, n_frames_lost == 0);
            break;
        } case bds_2_blink_off: {
            roi_size_cnt = 0;
            ItemTracker::track(time,ignore,additional_ignores);
            _blinking_drone_status = detect_blink(time, n_frames_tracking == 0);
            break;
        } case bds_2_blink_on: {
            roi_size_cnt = 0;
            ItemTracker::track(time,ignore,additional_ignores);
            _blinking_drone_status = detect_blink(time, n_frames_lost == 0);
            break;
        } case bds_3_blink_off_calib: {
            _visdat->enable_background_motion_map_calibration(bind_blink_time*0.8f);  //0.8 to prevent picking up the upcoming blink in the background calib
            _blinking_drone_status = bds_3_blink_off;
        } FALLTHROUGH_INTENDED; case bds_3_blink_off: {
            roi_size_cnt = 0;
            ItemTracker::track(time,ignore,additional_ignores);
            _blinking_drone_status = detect_blink(time, n_frames_tracking == 0);
            break;
        } case bds_3_blink_on: {
            roi_size_cnt = 0;
            ItemTracker::track(time,ignore,additional_ignores);
            _blinking_drone_status = detect_blink(time, n_frames_lost == 0);
            break;
        } case bds_dedicated_calib: {
            append_log();
#ifndef MANUAL_DRONE_LOCATE
            _blinking_drone_status = bds_found;
            break;
#endif
            _visdat->enable_background_motion_map_calibration(5.f);
            manual_calib_time_start = time;
            _blinking_drone_status = bds_calib_wait;
            break;
        } case bds_calib_wait: {
            append_log();
            if (time  - manual_calib_time_start > 5.1f)
                _blinking_drone_status = bds_found;
            break;
        } case bds_found: {
            _enable_depth_background_check = true;
            _enable_motion_background_check = true;
            append_log(); // no tracking needed in this stage
            _drone_tracking_status = dts_inactive; //progress to the next stage in the main tracker state machine

#ifdef MANUAL_DRONE_LOCATE
            _enable_roi = true;
            //TMP solution:
            _drone_blink_world_location.x = 0.190292642;
            _drone_blink_world_location.y = -1.64084888;
            _drone_blink_world_location.z = -1.32899487;
            //write to xml
            serialize_calib();
            break;
#endif
            //save found drone location
            _drone_blink_image_location = find_result.best_image_locationL.pt;
            track_data d = Last_track_data();
            _drone_blink_world_location.x = d.sposX;
            _drone_blink_world_location.y = d.sposY;
            _drone_blink_world_location.z = d.sposZ;
            if (!_drone_blink_world_location_start.x)
                _drone_blink_world_location_start = _drone_blink_world_location;

#ifndef MANUAL_DRONE_LOCATE
            if (!_landing_pad_location_set){
                _landing_pad_location_set = true;
                _landing_pad_image_location = _drone_blink_image_location;
                _landing_pad_world_location = _drone_blink_world_location;
            }
#endif

            _drone_control_predicted_image_location = _drone_blink_image_location;
            _drone_control_predicted_world_location = _drone_blink_world_location;

            _enable_depth_background_check = true;
            _enable_motion_background_check = true;

            //write to xml
            serialize_calib();
            break;
        }
        }
        break;
    case dts_inactive: {
        roi_size_cnt = 0; // don't grow roi in this stage
        start_take_off_time = time;
        predicted_pathL.clear();
        pathL.clear();
        foundL = false;
        find_result.best_image_locationL.pt = _drone_blink_image_location;
        predicted_locationL_last = _drone_blink_world_location;
        reset_tracker_ouput(time);
        _drone_control_prediction_valid = false;
        if (drone_is_active)
            _drone_tracking_status = dts_detecting_takeoff;
        else {
            ItemTracker::append_log(); //really no point in trying to detect the drone when it is inactive...
            break;
        }

    } FALLTHROUGH_INTENDED; case dts_detecting_takeoff: {
        insert_control_predicted_drone_location();
        ItemTracker::track(time,ignore,additional_ignores);
        ignores_for_insect_tracker.clear();
        ignores_for_insect_tracker.push_back(Drone_Startup_Im_Location());

#ifdef VIZ
        cv::Point2f tmpp = Drone_Startup_Im_Location();
        cv::circle(diff_viz,tmpp*IMSCALEF,1,cv::Scalar(255,0,0),1);
        cv::circle(diff_viz,Drone_Startup_Im_Location()*IMSCALEF,1,cv::Scalar(0,0,255),1);
        for (uint i = 0; i< wti.size(); i++){
            world_track_item k = wti.at(i);
            cv::circle(diff_viz,k.image_coordinates()*IMSCALEF,3,cv::Scalar(0,255,0),2);
        }
#endif
        if (!drone_is_active)
            _drone_tracking_status = dts_inactive;
        else if (n_frames_lost==0 && wti.size() > 1 ){
            bool takeoff_spot_detected = false;
            bool drone_detected_near_takeoff_spot = false;
            for (uint i = 0; i< wti.size(); i++){
                world_track_item k = wti.at(i);
                float dist2take_off = sqrt(pow(k.image_coordinates().x - Drone_Startup_Im_Location().x,2)+pow(k.image_coordinates().y - Drone_Startup_Im_Location().y,2));
                if (dist2take_off < settings.pixel_dist_landing_spot + DRONE_IM_START_SIZE){
                    takeoff_spot_detected = true;
                    ignores_for_insect_tracker.push_back(k.image_coordinates());
#ifdef MANUAL_DRONE_LOCATE
                    float disparity = stereo_match(k.image_coordinates(),_visdat->diffL,_visdat->diffR,find_result.disparity);
                    std::vector<cv::Point3d> camera_coordinates, world_coordinates;
                    camera_coordinates.push_back(cv::Point3d(k.image_coordinates().x*IMSCALEF,k.image_coordinates().y*IMSCALEF,-disparity));
                    cv::perspectiveTransform(camera_coordinates,world_coordinates,_visdat->Qf);
                    cv::Point3f output = world_coordinates[0];
                    float theta = _visdat->camera_angle * deg2rad;
                    float temp_y = output.y * cosf(theta) + output.z * sinf(theta);
                    output.z = -output.y * sinf(theta) + output.z * cosf(theta);
                    output.y = temp_y;
                    _drone_blink_world_location = output;
                    if (_landing_pad_location_set){
                        _landing_pad_location_set = true;
                        _landing_pad_image_location = _drone_blink_image_location;
                        _landing_pad_world_location = _drone_blink_world_location;
                    }
#endif
                } else if (dist2take_off > settings.pixel_dist_seperation_min + DRONE_IM_START_SIZE && dist2take_off < settings.pixel_dist_seperation_max + DRONE_IM_START_SIZE){
                    drone_detected_near_takeoff_spot = true;
                    ignores_for_insect_tracker.push_back(k.image_coordinates());
                } else if (dist2take_off < settings.pixel_dist_seperation_max + DRONE_IM_START_SIZE){
                    ignores_for_insect_tracker.push_back(k.image_coordinates());
                }
            }
            if (takeoff_spot_detected &&drone_detected_near_takeoff_spot ) {
                _drone_tracking_status = dts_detected;
                _visdat->delete_from_motion_map(Drone_Startup_Im_Location()*IMSCALEF, DRONE_IM_START_SIZE);
            }
        }
        roi_size_cnt = 0; // don't grow roi in this stage
        break;
    } case dts_detecting: {
        insert_control_predicted_drone_location();
        ItemTracker::track(time,ignore,additional_ignores);
        if (!drone_is_active)
            _drone_tracking_status = dts_inactive;
        else if (n_frames_lost==0)
            _drone_tracking_status = dts_detected;
        break;
    } case dts_detected: {
        insert_control_predicted_drone_location();
        ignores_for_insect_tracker.clear();
        ItemTracker::track(time,ignore,additional_ignores);
        if (!drone_is_active)
            _drone_tracking_status = dts_inactive;
        else if (!foundL)
            _drone_tracking_status = dts_detecting;
        break;
    }
    }


}

DroneTracker::blinking_drone_states DroneTracker::detect_blink(float time, bool found) {
    float blink_period = time - blink_time_start;
    if (found) {
        if ( blink_period > bind_blink_time - 0.1f && blink_period < bind_blink_time+0.1f) {
            blink_time_start = time;
            int tmp  =static_cast<int>(_blinking_drone_status)+1;
            return static_cast<blinking_drone_states>(tmp);
        } else {
            return bds_searching;
        }
    } else if (!found && blink_period > bind_blink_time +0.1f) {
        return bds_searching;
    }
    return _blinking_drone_status;
}

cv::Mat DroneTracker::get_probability_cloud(cv::Point size) {
    //TODO: make proper probability esitmate based on control inputs and movement estimates
    return createBlurryCircle(size);
}


/* Takes the calibrated uncertainty map, and augments it with a highlight around p */
cv::Mat DroneTracker::get_approx_cutout_filtered(cv::Point p, cv::Mat diffL_small, cv::Point size) {

    if (_enable_roi) {
        //calc roi:
        cv::Rect roi_circle(0,0,size.x,size.y);
        int x1 = p.x-size.x/2;
        if (x1 < 0) {
            roi_circle.x = abs(x1);
            roi_circle.width-=roi_circle.x;
        } else if (x1 + size.x >= diffL_small.cols)
            roi_circle.width = roi_circle.width  - abs(x1 + size.x - diffL_small.cols);

        int y1 = p.y-size.y/2;
        if (y1 < 0) {
            roi_circle.y = abs(y1);
            roi_circle.height-=roi_circle.y;
        } else if (y1 + size.y >= diffL_small.rows)
            roi_circle.height = roi_circle.height - abs(y1 + size.y - diffL_small.rows);

        cv::Mat blurred_circle = get_probability_cloud(size);
        cv::Mat cir = blurred_circle(roi_circle);
        _cir = cir;

        x1 = p.x-size.x/2+roi_circle.x;
        int x2 = roi_circle.width;
        y1 = p.y-size.y/2+roi_circle.y;
        int y2 = roi_circle.height;

        cv::Rect roi(x1,y1,x2,y2);
        find_result.roi_offset = roi;

        diffL_small(roi).convertTo(_dif, CV_32F);
        cv::Mat res;
        res = cir.mul(_dif);
        res.convertTo(res, CV_8UC1);

        return res;
    } else {
        find_result.roi_offset = cv::Rect(0,0,diffL_small.cols,diffL_small.rows);
        return diffL_small;
    }
}

void DroneTracker::deserialize_calib(std::string file) {
    std::cout << "Reading calibration from: " << file << std::endl;
    std::ifstream infile(file);

    std::string xmlData((std::istreambuf_iterator<char>(infile)),
                        std::istreambuf_iterator<char>());


    DroneTrackerCalibrationData* dser=new DroneTrackerCalibrationData; // Create new object
    if (xmls::Serializable::fromXML(xmlData, dser)) // perform deserialization
    { // Deserialization successful
        _drone_blink_image_location.x = dser->Drone_startup_image_location_x.value();
        _drone_blink_image_location.y = dser->Drone_startup_image_location_y.value();
        _drone_blink_world_location.x = dser->Drone_startup_world_location_x.value();
        _drone_blink_world_location.y = dser->Drone_startup_world_location_y.value();
        _drone_blink_world_location.z = dser->Drone_startup_world_location_z.value();
    } else { // Deserialization not successful
        throw my_exit("cannot read drone tracker calibration file.");
    }
}

void DroneTracker::serialize_calib() {
    DroneTrackerCalibrationData *calib_settings=new DroneTrackerCalibrationData; // Create new object
    calib_settings->Drone_startup_image_location_x = _drone_blink_image_location.x;
    calib_settings->Drone_startup_image_location_y = _drone_blink_image_location.y;

    calib_settings->Drone_startup_world_location_x = _drone_blink_world_location.x;
    calib_settings->Drone_startup_world_location_y = _drone_blink_world_location.y;
    calib_settings->Drone_startup_world_location_z = _drone_blink_world_location.z;

    std::string xmlData = calib_settings->toXML();
    std::ofstream outfile(calib_fn);
    outfile << xmlData ;
    outfile.close();
}
