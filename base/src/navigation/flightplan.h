#pragma once
#include "common.h"

namespace navigation {
enum waypoint_flight_modes {
    wfm_takeoff,
    wfm_flying,
    wfm_flower,
    wfm_brick,
    wfm_wp_stay,
    wfm_landing,
    wfm_yaw_reset,
    wfm_thrust_calib,
    wfm_long_range
};
static const char *waypoint_flight_modes_str[] = {
    "wfm_takeoff",
    "wfm_flying",
    "wfm_flower",
    "wfm_brick",
    "wfm_wp_stay",
    "wfm_landing",
    "wfm_yaw_reset",
    "wfm_thrust_calib",
    "wfm_long_range",
    "" // must be the last entry! (check in serializer)
};
const int hover_mode_wp_dist_threshold = 300;
struct Waypoint {
    Waypoint(cv::Point3f p, int distance_threshold_mm, float vel_thresh, float hover_pause_time, std::string wp_name) {
        xyz = p;
        name = wp_name;
        threshold_mm = distance_threshold_mm;
        threshold_v = vel_thresh;
        hover_pause = hover_pause_time;
        mode  = wfm_flying;
    }
    cv::Point3f xyz = {0};
    int threshold_mm = 50;
    float threshold_v = 1.7;
    float hover_pause = 0;
    float pitch_duration = 0;
    waypoint_flight_modes mode;
    std::string name;
protected:
    Waypoint() {}
};
struct Waypoint_Landing : Waypoint {
    Waypoint_Landing() {
        xyz = cv::Point3f(0, 0.5f, 0); // only for landing wp, relative to the startup location!
        threshold_mm = 10;
        threshold_v = 0.15f;
        hover_pause = 3;
        mode = wfm_landing;
        name = "wp_land";
    }
};
struct Waypoint_Yaw_Reset : Waypoint {
    Waypoint_Yaw_Reset() {
        xyz = cv::Point3f(0, 0.5f, 0);
        threshold_mm = hover_mode_wp_dist_threshold;
        threshold_v = 2.0f;
        hover_pause = 1;
        mode = wfm_yaw_reset;
        name = "wp_yaw";
    }
};
struct Waypoint_Thrust_Calibration : Waypoint {
    Waypoint_Thrust_Calibration() {
        xyz = cv::Point3f(0, 0.5f, 0);
        threshold_mm = 15;
        threshold_v = 0.15f;
        hover_pause = 5;
        mode = wfm_thrust_calib;
        name = "wp_thrust_calib";
    }
};
struct Waypoint_Takeoff : Waypoint {
    Waypoint_Takeoff() {
        xyz = cv::Point3f(0, 1.f, 0); // takeoff direction
        mode = wfm_takeoff;
        name = "wp_takeoff";
    }
};
struct Waypoint_Flower : Waypoint {
    Waypoint_Flower(cv::Point3f p, std::string wp_name) : Waypoint(p, 0, 0, 0, wp_name) {
        mode = wfm_flower;
    }
};
struct Waypoint_Brick : Waypoint {
    Waypoint_Brick(cv::Point3f p, std::string wp_name) : Waypoint(p, 0, 0, 0, wp_name) {
        mode = wfm_brick;
    }
};
struct Waypoint_Stay : Waypoint {
    Waypoint_Stay(cv::Point3f p, std::string wp_name) : Waypoint(p, 0, 0, 0, wp_name) {
        mode = wfm_wp_stay;
    }
};
struct Waypoint_Long_Range : Waypoint {
    Waypoint_Long_Range(cv::Point3f p, std::string wp_name, float pitch_duration_) : Waypoint(p, 0, 0, 0, wp_name) {
        mode = wfm_long_range;
        pitch_duration = pitch_duration_;
    }
};

class XML_Waypoint_Mode: public xmls::MemberBase
{
private:
    void AssignValue(const waypoint_flight_modes value) {
        m_sValue = waypoint_flight_modes_str[value];
    };
public:
    XML_Waypoint_Mode() {AssignValue(static_cast<waypoint_flight_modes>(0));};
    XML_Waypoint_Mode(waypoint_flight_modes value) {AssignValue(value);};
    waypoint_flight_modes value() {
        string sHelp =  m_sValue;
        transform(sHelp.begin(), sHelp.end(), sHelp.begin(), ::tolower);

        for (uint i = 0; waypoint_flight_modes_str[i] != string(""); i++) {
            if (waypoint_flight_modes_str[i] == sHelp)
                return static_cast<waypoint_flight_modes>(i);
        }
        throw MyExit("wrong waypoint_flight_mode: " + sHelp);
    };

    XML_Waypoint_Mode operator=(const waypoint_flight_modes value) {AssignValue(value); return *this;};
};

class XML_Waypoint: public xmls::Serializable
{
public:
    XML_Waypoint_Mode mode;
    xmls::xInt threshold_mm;
    xmls::xFloat threshold_v;
    xmls::xFloat hover_pause;
    xmls::xFloat pitch_duration;
    xmls::xFloat x;
    xmls::xFloat y;
    xmls::xFloat z;
    xmls::xString name;

    XML_Waypoint() {
        setClassName("Waypoint");
        setVersion("1.3"); // not used at the moment
        Register("type", &mode);
        Register("threshold_mm", &threshold_mm);
        Register("threshold_v", &threshold_v);
        Register("hover_pause", &hover_pause);
        Register("pitch_duration", &pitch_duration);
        Register("x", &x);
        Register("y", &y);
        Register("z", &z);
        Register("name", &name);
    }
    XML_Waypoint(Waypoint wp) : XML_Waypoint() {
        mode = wp.mode;
        if (wp.mode != wfm_landing && wp.mode != wfm_takeoff) {
            x = wp.xyz.x;
            y = wp.xyz.y;
            z = wp.xyz.z;
            name = wp.name;
            threshold_mm = wp.threshold_mm;
            threshold_v = wp.threshold_v;
            hover_pause = wp.hover_pause;
        }
        if (wp.mode == wfm_long_range)
            pitch_duration = static_cast<Waypoint_Long_Range *>(&wp)->pitch_duration;
        else
            pitch_duration = 0;
    }

    Waypoint waypoint() {
        switch (mode.value()) {
            case waypoint_flight_modes::wfm_takeoff: {
                    Waypoint_Takeoff wp;
                    return wp;
                    break;
            } case waypoint_flight_modes::wfm_landing: {
                    Waypoint_Landing wp;
                    return wp;
                    break;
            } case waypoint_flight_modes::wfm_wp_stay: {
                    Waypoint_Stay wp(cv::Point3f(x.value(), y.value(), z.value()), name.value());
                    return wp;
                    break;
            } case waypoint_flight_modes::wfm_yaw_reset: {
                    Waypoint_Yaw_Reset wp;
                    return wp;
                    break;
            } case waypoint_flight_modes::wfm_thrust_calib: {
                    Waypoint_Thrust_Calibration wp;
                    return wp;
                    break;
            } case waypoint_flight_modes::wfm_brick: {
                    Waypoint_Brick wp(cv::Point3f(x.value(), y.value(), z.value()), name.value());
                    return wp;
                    break;
            } case waypoint_flight_modes::wfm_flower: {
                    Waypoint_Flower wp(cv::Point3f(x.value(), y.value(), z.value()), name.value());
                    return wp;
                    break;
            } case waypoint_flight_modes::wfm_long_range: {
                    Waypoint_Long_Range wp(cv::Point3f(x.value(), y.value(), z.value()), name.value(), pitch_duration.value());
                    return wp;
                    break;
            } case waypoint_flight_modes::wfm_flying:
            default: {
                    Waypoint wp(cv::Point3f(x.value(), y.value(), z.value()), threshold_mm.value(), threshold_v.value(), hover_pause.value(), name.value());
                    return wp;
                    break;
                }
        }
    }
};

class XML_FlightPlan: public xmls::Serializable
{

public:
    xmls::Collection<XML_Waypoint> waypointsxml;
    xmls::xString flightplan_name;

    XML_FlightPlan() {
        setClassName("FlightPlanXML");
        setVersion("1.3");
        Register("flightplan_name", &flightplan_name);
        Register("waypoints", &waypointsxml);
    }
    XML_FlightPlan(std::string name, std::vector<Waypoint> wps) : XML_FlightPlan() {
        flightplan_name = name;
        for (auto wp : wps) {
            XML_Waypoint *wpxml = new XML_Waypoint(wp);
            waypointsxml.addItem(wpxml);
        }
    }

    void deserialize(std::string filepath) {
        std::cout << "Reading settings from: " << filepath << std::endl;
        if (file_exist(filepath)) {
            std::ifstream infile(filepath);
            std::string xmlData((std::istreambuf_iterator<char>(infile)),
                                std::istreambuf_iterator<char>());

            if (!Serializable::fromXML(xmlData, this))
            {   // Deserialization not successful
                throw MyExit("Cannot read: " + filepath);
            }
            XML_FlightPlan tmp;
            auto v1 = getVersion();
            auto v2 = tmp.getVersion();
            if (v1 != v2) {
                throw MyExit("XML version difference detected from " + filepath);
            }
            infile.close();
        } else {
            throw MyExit("File not found: " + filepath);
        }
    }

    void serialize(std::string filepath) {
        std::string xmlData = toXML();
        std::ofstream outfile = std::ofstream(filepath);
        outfile << xmlData ;
        outfile.close();

    }
    std::vector<Waypoint> waypoints() {
        std::vector<Waypoint> res;
        for (uint i = 0; i < waypointsxml.size(); i++) {
            res.push_back(waypointsxml.getItem(i)->waypoint());
        }
        return res;
    }
};

}
