#ifndef JSON_STRUCTURE_G
#define JSON_STRUCTURE_G
#include "nlohmann/json.hpp"
#include <string>
#include <LinearMath/btVector3.h>

namespace JSON_Structure
{
    namespace{
        nlohmann::json jsonFile;
    }

    void setup();
    void setup(nlohmann::json jsonFile);
    void setRestCableLength(int num, double val);
    void setCurrentCableLength(int num, double val);

    void setEndPoints(int num, btVector3 end_point);
    void setLegEndPoints(int num, btVector3 end_point);
    void setEndPointVelocity(int num, btVector3 end_point_velocity);

    void setFlags(int index, int value);
    void setTime(double t);
    std::string jsonToString();
    nlohmann::json stringToJson(char *s);

}


#endif