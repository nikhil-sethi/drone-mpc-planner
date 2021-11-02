
#include "visualizer3d.h"

#ifdef VIZ_3D

#include <fastrtps/Domain.h>
#include <ctime>

using namespace eprosima;
using namespace eprosima::fastrtps;

bool Visualizer3D::init(Patser *patser)
{
    bool init_success = true;

    _dtrkr = trackers->dronetracker();
    _itrkr = trackers->target_insecttracker();
    _dctrl = dctrl;
    _dnav = dnav;

    pub_tf.add_transform(0, 0, 2.0, 0, 1, 1, 0, _time, "world", "pats_frame", true);
    pub_tf.add_transform(0, 0, 0, -0.259, 0, 0, 0.966, _time, "pats_frame", "camera", true);

    init_success &= pub_tf.init();
    init_success &= pub_path.init(cam_volume);

    return init_success;
}

void Visualizer3D::run()
{
    _time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    static int count = 0;
    count ++;

    addDrone();
    addInsect();
    addTarget();

    pub_tf.run();
    pub_path.run(_time);

    pub_tf.clear_transforms();
}

void Visualizer3D::addDrone()
{
    TrackData data = _dtrkr->last_track_data();

    if (data.pos_valid) {
        pub_tf.add_transform(static_cast<double>(data.pos().x), static_cast<double>(data.pos().y), static_cast<double>(data.pos().z), 0, 0, 0, 1, _time, "pats_frame", "hunter");
        pub_path.add_drone_pos(static_cast<double>(data.pos().x), static_cast<double>(data.pos().y), static_cast<double>(data.pos().z), _time);
    }
}

void Visualizer3D::addInsect()
{
    TrackData data = _itrkr->last_track_data();

    if (data.pos_valid && data.pos().x != 0.0f)
    {
        pub_tf.add_transform(static_cast<double>(data.pos().x), static_cast<double>(data.pos().y), static_cast<double>(data.pos().z), 0, 0, 0, 1, _time, "pats_frame", "insect");
        pub_path.add_insect_pos(static_cast<double>(data.pos().x), static_cast<double>(data.pos().y), static_cast<double>(data.pos().z), _time);
    }
}

void Visualizer3D::addTarget()
{
    TrackData data = _itrkr->last_track_data();

    if (_dctrl->Joy_State() != DroneController::js_hunt) {

        pub_path.set_target(_dnav->setpoint().pos().x, _dnav->setpoint().pos().y, _dnav->setpoint().pos().z, _time);
    } else {
        if (data.pos_valid) {
            pub_path.set_target(static_cast<double>(data.pos().x), static_cast<double>(data.pos().y), static_cast<double>(data.pos().z), _time);
        }
    }
}

#endif

Visualizer3D::Visualizer3D()
{

}
