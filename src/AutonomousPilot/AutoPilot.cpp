#include "AutoPilot.hpp"


unsigned int AutoPilot::compute_roll(const float &current_roll) {
    unsigned int angle = (unsigned int) (-(current_roll) * (2000 - 1000)/(180) + 1500);
    angle = (angle < 1000 ? 1000 : angle);
    angle = (angle > 2000 ? 2000 : angle);
    return angle;
}

unsigned int AutoPilot::compute_pitch(MissionPlanner &MP, const float &pitch, const double &alt_var, const GPSdata &gps_container) {
    WayPointData targetWP = MP.getCurrentWP();
    float a = (alt_var < targetWP.altitude ? targetWP.altitude - alt_var : alt_var - targetWP.altitude);

    WayPointData currentWP{gps_container.N, gps_container.E, alt_var, 0};
    float b = (float) MP.flatDistance(currentWP);
    
    double flightPitchAttackAngle = (atan(b / a) * 180 / M_PI) - pitch;
    uint angle = (uint) ((alt_var < targetWP.altitude ? -flightPitchAttackAngle : flightPitchAttackAngle) * (2000 - 1000)/(180) + 1500);
    
    angle = (angle < 1000 ? 1000 : angle);
    angle = (angle > 2000 ? 2000 : angle);
    
    return angle;
}

unsigned int AutoPilot::compute_yaw(MissionPlanner &MP, const IMUdata &imu_contaier, const GPSdata &gps_container) {
    WayPointData currentWP{gps_container.N, gps_container.E, 0, 0};
    double angle = MP.calculateInitialBearingAndRequiredYaw(currentWP, imu_contaier.yaw);
    uint pwmAngle = (uint) (angle * ((2000 - 1000) / 180)) + 1500; 

    pwmAngle = (pwmAngle < 1000 ? 1000 : pwmAngle);
    pwmAngle = (pwmAngle > 2000 ? 2000 : pwmAngle);
 
    pwmAngle = 3000 - pwmAngle;

    return pwmAngle;
}

unsigned int AutoPilot::compute_throttle(MissionPlanner &MP, const double &alt_var, const GPSdata &gps_container) {
    float currentSpeed = gps_container.speed;
    WayPointData currentLocation{gps_container.N, gps_container.E, alt_var, gps_container.speed};
    double distanceFromPrevWP = sqrt(pow(MP.flatDistance(currentLocation, -1), 2) + pow(std::fabs(MP.getPrevWP().altitude - alt_var), 2));
    double distanceFromNextWP = sqrt(pow(MP.flatDistance(currentLocation), 2) + pow(std::fabs(MP.getCurrentWP().altitude - alt_var), 2));
    double distance = distanceFromPrevWP + distanceFromNextWP;
    float requiredSpeed = (((distance - distanceFromPrevWP) * MP.getPrevWP().speed) / distance) + (((distance - distanceFromNextWP) * MP.getCurrentWP().speed) / distance);

    double output = ((requiredSpeed - currentSpeed) * 100) + min_pwm;
    output = (output < 1000 ? 1000 : output);
    output = (output > 2000 ? 2000 : output);

    return output;
}