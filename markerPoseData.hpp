//
// Created by luigi on 7/15/21.
//

#ifndef EXAMPLESUBSCRIBER_MARKERPOSEDATA_HPP
#define EXAMPLESUBSCRIBER_MARKERPOSEDATA_HPP

#include <json.hpp>
#include <stdio.h>

struct markerPoseData
{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;



    void to_string(std::string& out)
    {
        std::stringstream out_stream;
        out_stream << "MARKER " << x << ", " << y << "," << z << ", " << roll << ", " << pitch << ", " << yaw;
        out = out_stream.str();

    }

    void to_json(nlohmann::json &j)
    {
        j["x"] = x;
        j["y"] = y;
        j["z"] = z;
        j["roll"] = roll;
        j["pitch"] = pitch;
        j["yaw"] = yaw;

    }

    bool from_json(const nlohmann::json &j) {

        try {
            x = j["x"] ;
            y = j["y"] ;
            z = j["z"] ;
            roll = j["roll"];
            pitch = j["pitch"] ;
            yaw = j["yaw"] ;

        } catch (std::exception &e) {
            return false;
        }
        return true;
    }
};

#endif //EXAMPLESUBSCRIBER_MARKERPOSEDATA_HPP



































