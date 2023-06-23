#include <string>

class CelestialBody
{
public:
    CelestialBody(
        const std::string &name,
        double mass,
        double distance,
        double orbit_period,
        double spin_period,
        double inclination);

    void update(double current_time);

private:
    double _mass;
    double _distance;
    double _orbit_period;
    double _spin_period;
    double _inclination;

    double _current_orbit_angle;
    double _current_spin_angle;
};
