#ifndef AUTO_PILOT
#define AUTO_PILOT

#include "../drivers/IMUdriver/IMUdriver.hpp"
#include "../drivers/BAROdriver/driver/bmp3_defs.h"
#include "../drivers/GPSdriver/GPSdriver.hpp"
#include "MissionPlanner.hpp"

class AutoPilot {
    private:
        // Initialize variables
        const uint min_pwm = 1000;

    public:
        unsigned int compute_roll(const float &current_roll);

        unsigned int compute_pitch(MissionPlanner &MP, const float &pitch, const double &alt_var, const GPSdata &gps_container);

        unsigned int compute_yaw(MissionPlanner &MP, const IMUdata &imu_contaier, const GPSdata &gps_container);

        unsigned int compute_throttle(MissionPlanner &MP, const double &alt_var, const GPSdata &gps_container);
};

#endif