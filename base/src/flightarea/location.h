#pragma once
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "common.h"
#include "plane.h"

class XML_Plane_Type: public xmls::MemberBase {
private:
    void AssignValue(const plane_types value) {
        m_sValue = plane_types_str[value];
    };
public:
    XML_Plane_Type() {AssignValue(static_cast<plane_types>(0));};
    XML_Plane_Type(plane_types value) {AssignValue(value);};
    plane_types value() {
        string sHelp =  m_sValue;
        transform(sHelp.begin(), sHelp.end(), sHelp.begin(), ::tolower);

        for (uint i = 0; plane_types_str[i] != string(""); i++) {
            if (plane_types_str[i] == sHelp)
                return static_cast<plane_types>(i);
        }
        throw MyExit("wrong plane type: " + sHelp);
    };

    XML_Plane_Type operator=(const plane_types value) {AssignValue(value); return *this;};
};

class XML_Plane: public xmls::Serializable
{
private:
    enum definition_type {
        vector_based,
        point_based,
        angle_based
    };

    definition_type identify_definition_type() {
        std::vector<float> vector_based_plane_params = {
            support_vector_x.value(),
            support_vector_y.value(),
            support_vector_z.value(),
            normal_vector_x.value(),
            normal_vector_y.value(),
            normal_vector_z.value()
        };
        std::vector<float> point_based_plane_params = {
            direction.value(),
            point1_x.value(),
            point1_y.value(),
            point1_z.value(),
            point2_x.value(),
            point2_y.value(),
            point2_z.value(),
            point3_x.value(),
            point3_y.value(),
            point3_z.value()
        };
        std::vector<float> angle_based_plane_params = {
            distance.value(),
            roll_deg.value(),
            pitch_deg.value()
        };
        if (!std::equal(vector_based_plane_params.begin() + 1, vector_based_plane_params.end(), vector_based_plane_params.begin()))
            return vector_based;
        else if (!std::equal(point_based_plane_params.begin() + 1, point_based_plane_params.end(), point_based_plane_params.begin()))
            return point_based;
        else if (!std::equal(angle_based_plane_params.begin() + 1, angle_based_plane_params.end(), angle_based_plane_params.begin()))
            return angle_based;
        else
            throw std::runtime_error("Plane configuration is empty");
    }


public:
    XML_Plane_Type type;
    xmls::xFloat support_vector_x;
    xmls::xFloat support_vector_y;
    xmls::xFloat support_vector_z;
    xmls::xFloat normal_vector_x;
    xmls::xFloat normal_vector_y;
    xmls::xFloat normal_vector_z;

    xmls::xFloat direction;
    xmls::xFloat point1_x;
    xmls::xFloat point1_y;
    xmls::xFloat point1_z;
    xmls::xFloat point2_x;
    xmls::xFloat point2_y;
    xmls::xFloat point2_z;
    xmls::xFloat point3_x;
    xmls::xFloat point3_y;
    xmls::xFloat point3_z;

    xmls::xFloat distance;
    xmls::xFloat roll_deg;
    xmls::xFloat pitch_deg;

    XML_Plane() {
        setClassName("Plane");
        setVersion("1.0"); // not used at the moment
        Register("type", &type);
        Register("support_vector_x", &support_vector_x);
        Register("support_vector_y", &support_vector_y);
        Register("support_vector_z", &support_vector_z);
        Register("normal_vector_x", &normal_vector_x);
        Register("normal_vector_y", &normal_vector_y);
        Register("normal_vector_z", &normal_vector_z);

        Register("direction", &direction);
        Register("point1_x", &point1_x);
        Register("point1_y", &point1_y);
        Register("point1_z", &point1_z);
        Register("point2_x", &point2_x);
        Register("point2_y", &point2_y);
        Register("point2_z", &point2_z);
        Register("point3_x", &point3_x);
        Register("point3_y", &point3_y);
        Register("point3_z", &point3_z);

        Register("distance", &distance);
        Register("roll_deg", &roll_deg);
        Register("pitch_deg", &pitch_deg);
    }

    XML_Plane(Plane plane) : XML_Plane() {
        type = plane.type;
        support_vector_x = plane.support.x;
        support_vector_y = plane.support.y;
        support_vector_z = plane.support.z;
        normal_vector_x = plane.normal.x;
        normal_vector_y = plane.normal.y;
        normal_vector_z = plane.normal.z;
    }

    Plane plane() {
        definition_type def_typ = identify_definition_type();
        if (def_typ == vector_based) {
            std::cout << "Vector based plane definition" << std::endl;
            return Plane(support_vector_x.value(), support_vector_y.value(), support_vector_z.value(),
                         normal_vector_x.value(), normal_vector_y.value(), normal_vector_z.value(), type.value());
        } else if (def_typ == point_based) {
            std::cout << "Point based plane definition" << std::endl;
            return Plane(direction.value(), cv::Point3f(point1_x.value(), point1_y.value(), point1_z.value()),
                         cv::Point3f(point2_x.value(), point2_y.value(), point2_z.value()),
                         cv::Point3f(point3_x.value(), point3_y.value(), point3_z.value()),
                         type.value());
        } else if (def_typ == angle_based) {
            std::cout << "Angle based plane definition" << std::endl;
            return Plane(distance.value(), roll_deg.value(), pitch_deg.value(), type.value());
        } else {
            throw std::runtime_error("Plane configuration invalid: Normal vector has no direction");
        }
    }
};

class XML_Location: public xmls::Serializable
{

public:
    xmls::xString _name;
    xmls::Collection<XML_Plane> planes_xml;

    XML_Location() {
        setClassName("LocationXML");
        setVersion("1.0");
        Register("name", &_name);
        Register("planes", &planes_xml);
    }
    XML_Location(std::string name, std::vector<Plane> planes) : XML_Location() {
        _name = name;
        for (auto plane : planes) {
            XML_Plane *plane_xml = new XML_Plane(plane);
            planes_xml.addItem(plane_xml);
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
            XML_Location tmp;
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
    std::vector<Plane> planes() {
        std::vector<Plane> res;
        for (uint i = 0; i < planes_xml.size(); i++) {
            res.push_back(planes_xml.getItem(i)->plane());
        }
        return res;
    }
};
