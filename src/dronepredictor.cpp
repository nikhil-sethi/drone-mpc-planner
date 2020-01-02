#include "dronepredictor.h"
#include "common.h"

void DronePredictor::init(VisionData *visdat, tracking::DroneTracker *dtrk, tracking::InsectTracker *itrk, DroneController *dctrl) {
    _dctrl = dctrl;
    _dtrk = dtrk;
    _itrk = itrk;
    _visdat = visdat;
    throttle_smth.init(1,0);
    roll_smth.init(1,0);
    pitch_smth.init(1,0);

    swap_x.init(pparams.fps,0);
    swap_y.init(pparams.fps,0);
    swap_z.init(pparams.fps,0);

    drag_gain = cv::Point3f(0.01f*0.2f,0.01f,0.01f*0.2f);

    throttle_gain_smth.init(300,10); // full stick -> 10 m/s² acceleration
    roll_gain_smth.init(300,-10);
    pitch_gain_smth.init(300,-10);
}

cv::Point3f mult3f(cv::Point3f p1, cv::Point3f p2){
    cv::Point3f p;
    p.x = p1.x*p2.x;
    p.y = p1.y*p2.y;
    p.z = p1.z*p2.z;
    return p;
}
cv::Point3f div3f(cv::Point3f p1, cv::Point3f p2){
    cv::Point3f p;
    p.x = p1.x/p2.x;
    p.y = p1.y/p2.y;
    p.z = p1.z/p2.z;
    return p;
}

void DronePredictor::update(bool drone_is_active, double time) {

    if (_dtrk->track_history.size()==0)
        return;

    //smooth control inputs  (which also sort of accounts for the mass of the drone):
    float throttle = throttle_smth.addSample(_dctrl->Throttle());
    float pitch = pitch_smth.addSample(_dctrl->Pitch());
    float roll = roll_smth.addSample(_dctrl->Roll());

    if (fabs(roll) > fabs(pitch))
        throttle -= dparams.throttle_bank_factor*fabs(roll);
    else
        throttle -= dparams.throttle_bank_factor*fabs(pitch);
    if (throttle< -1)
        throttle = -1;

    cv::Point3f rtp(roll,throttle,pitch);
    swap_check(rtp);

    track_data td = _dtrk->track_history.back();


    for (uint i = 0; i<_dtrk->track_history.size();i++) {
        td = _dtrk->track_history.at(_dtrk->track_history.size()-i-1);
        if (td.pos_valid ) // || i > pparams.fps
            break;
    }
    if (!td.pos_valid)
        return;

    double dt_pred = (1./pparams.fps)+ time -td.time;
    if (dt_pred > 1) // don't try to predict based on control more then 1 second
        return;

    cv::Point3f pos = td.state.pos;


    if (drone_is_active && initialized && dt_prev < 1.f / pparams.fps && !_dtrk->taking_off() && td.acc_valid){
        //update gain_factors based on error in old prediciton

        float hdt2 = (0.5f*powf(dt_prev,2));

        cv::Point3f dx = pos-pos_prev;
        cv::Point3f dx_v = vel_prev * dt_prev;
        cv::Point3f dx_a = dx - dx_v;

        cv::Point3f measured_dacc = dx_a / hdt2;

        //calc ratio between predicted acc, and measured acc
        cv::Point3f ra1 = measured_dacc + mult3f(vel_prev,drag_gain);
        cv::Point3f ra2 = acc_pred_prev + mult3f(vel_prev,drag_gain);
        cv::Point3f ratio = div3f(ra1 , ra2);
        //calc what the gain should have been to result in the actual pos
        cv::Point3f new_gain = mult3f(ratio,rtp_gain_prev);

        /*
        //check check:
        //if the control input was close to zero, prevent divide by zeros
        if (fabs(rtp_prev.x) < 0.05f)
            new_gain.x = 1;
        if (fabs(rtp_prev.y) < 0.05f)
            new_gain.y = 1;
        if (fabs(rtp_prev.z) < 0.05f)
            new_gain.z = 1;

        cv::Point3f new_estimated_acc = (mult3f(rtp_prev ,new_gain) - mult3f(vel_prev,drag_gain));
        cv::Point3f updated_pos = pos_prev + new_estimated_acc * hdt2  + vel_prev * dt_prev;

        std::cout << "diff: " << updated_pos-pos << std::endl; // should be zero
        std::cout << "rtp_gain_err: " << new_gain.y << " throttle: " << throttle << std::endl;
        */

            //prevent infs if control input is close to 0
            if (fabs(rtp_prev.x) > 0.1f)
                roll_gain_smth.addSample(new_gain.x);
        if (fabs(rtp_prev.y) > 0.3f)
            throttle_gain_smth.addSample(new_gain.y);
        if (fabs(rtp_prev.z) > 0.3f)
            pitch_gain_smth.addSample(new_gain.z);
    }

    //estimate position based on current position and speed, and control inputs
    cv::Point3f rtp_gain = cv::Point3f(roll_gain_smth.latest(),throttle_gain_smth.latest(),pitch_gain_smth.latest());
    cv::Point3f vel = td.vel();

    //predict position based on control inputs and current position and velocity
    cv::Point3f acc_pred = mult3f(rtp ,rtp_gain) - mult3f(vel ,drag_gain);

    if (_dtrk->taking_off())
        acc_pred = cv::Point3f(0,20,0); // The target is not tracked properly yet, so override acc when taking off. Assume 2g take off.

    predicted_pos = pos + vel*dt_pred;// + 0.5*acc_pred * powf(dt_pred,2); TODO: improve prediciton by including Accelleration term
    if (_dtrk->taking_off())
        predicted_pos.y+=0.05f; //look for the drone a bit higher then the current position (to favor the drone instead of the landing pad)

    cv::Point2f image_location = world2im_2d(predicted_pos,_visdat->Qfi,_visdat->camera_angle);
    image_location /= pparams.imscalef;

    if (image_location.x < 0)
        image_location.x = 0;
    else if (image_location.x >= IMG_W/pparams.imscalef)
        image_location.x = IMG_W/pparams.imscalef-1;
    if (image_location.y < 0)
        image_location.y = 0;
    else if (image_location.y >= IMG_H/pparams.imscalef)
        image_location.y = IMG_H/pparams.imscalef-1;

    _dtrk->control_predicted_drone_location(image_location,predicted_pos); //TODO: activate this again
    //    _dtrk->predicted_locationL_prev = _dtrk->predicted_locationL_last;
    //    _dtrk->predicted_locationL_last.x = image_location.x;
    //    _dtrk->predicted_locationL_last.y = image_location.y;


    //save data for determining error next iteration:
    pos_prev = pos;
    vel_prev = vel;
    rtp_prev = rtp;
    acc_pred_prev = acc_pred;
    rtp_gain_prev = rtp_gain;
    dt_prev = dt_pred;

    if (drone_is_active && dt_prev < 5.f / pparams.fps)
        initialized = true;
}

void DronePredictor::swap_check(cv::Point3f rtp) {

    //calc the cumulative error between the control inputs and the acceleration of the items

    if (!_dctrl->drone_is_active())
        return;

    uint n = _dtrk->track_history.size();
    if (n != _itrk->track_history.size() || n != _dctrl->control_history.size())
        return;

    track_data td = _dtrk->track_history.back();
    track_data ti = _itrk->track_history.back();

    if (!td.acc_valid || !ti.acc_valid)
        return;

    tot_d+=td.acc();
    tot_i+=ti.acc();
    tot_t+=mult3f(rtp,rtp_gain_prev);

    float res_x = swap_x.addSample(fabs(tot_t.x - tot_d.x ) / fabs(tot_t.x -tot_i.x));
    float res_y = swap_y.addSample(fabs(tot_t.y - tot_d.y ) / fabs(tot_t.y -tot_i.y));
    //    float res_z = swap_z.addSample(fabs(tot_t.z - tot_d.z ) / fabs(tot_t.z -tot_i.z));
    if ((res_y > 10.f && tot_t.y > 250) || (res_x > 10.f && tot_t.x > 250))
        std::cout << "swapped?" << std::endl;

    //    std::cout << time << ", " << res_x << ", "  << res_y << ", "  << res_z
    //              << ", "  << td.saccX << ", " << ti.saccX << ", " << rtp.x*rtp_gain_prev.x
    //              << ", "  << td.saccY << ", " << ti.saccY << ", " << rtp.y*rtp_gain_prev.y
    //              << ", "  << td.saccZ << ", " << ti.saccZ << ", " << rtp.z*rtp_gain_prev.z << std::endl;

}
