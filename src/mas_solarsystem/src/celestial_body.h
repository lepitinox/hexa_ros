// h file of CelestialBody class

#ifndef CELESTIAL_BODY_H
#define CELESTIAL_BODY_H

#include <string>

class CelestialBody {
public:
    CelestialBody(const std::string &name, double mass, std::vector<double> position, std::vector<double> velocity, double g_constant = 6.67430e-11, const std::string &publish_topic = "position")
        : name_(name), mass_(mass), position_(position), velocity_(velocity), g_constant_(g_constant), publish_topic_(publish_topic)
    {
    }

    std::string name_;
    double mass_;
    std::vector<double> position_;
    std::vector<double> velocity_;
    double g_constant_;
    std::string publish_topic_;
};

#endif // CELESTIAL_BODY_H
