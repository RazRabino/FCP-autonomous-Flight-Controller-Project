#ifndef MISSION_PLANNER
#define MISSION_PLANNER

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

typedef struct {
    double N;
    double E;
    double altitude;
    float speed;
} WayPointData;

class MissionPlanner {
    private:
        std::vector<WayPointData> missionRoute;
        int currentWP;
        const double inZoneDistanceCheck = 60;
        
        const double a = 6378137.0;
        const double b = 6356752.314;
        const double f = 1 / 298.257223563;

        double degreesToRadians(double degrees) {
            return degrees * M_PI / 180.0;
        }

    public:
        MissionPlanner(std::string &missionWayPointsStr);

        bool isInWPZone(const WayPointData &sensorsData);

        double flatDistance(const WayPointData &sensorsData, const int wpIndex = 0);

        double calculateInitialBearingAndRequiredYaw(const WayPointData &sensorData, const float &yawAngle);

        void setNextWP() {this->currentWP++;}

        WayPointData getCurrentWP() {
            if(this->currentWP != -1 && this->currentWP < this->missionRoute.size()) return this->missionRoute[this->currentWP];
            else return WayPointData{-1.0, -1.0, -1.0, -1.0};
        }

        WayPointData getNextWP() {
            if(this->currentWP != -1 && currentWP + 1 < this->missionRoute.size()) return this->missionRoute[this->currentWP + 1];
            else return WayPointData{-1.0, -1.0, -1.0, -1.0};
        }

        WayPointData getPrevWP() {
            if(this->currentWP != -1 && currentWP != 0) return this->missionRoute[this->currentWP - 1];
            else return WayPointData{-1.0, -1.0, -1.0, -1.0};
        }

        WayPointData getTakeOffWP() {return this->missionRoute[0];}

        WayPointData getLandingWP() {return this->missionRoute[this->missionRoute.size() - 1];}

        bool isInTakeOff() {return this->currentWP == 1 ? true : false;}

        bool isInLanding() {return this->currentWP == this->missionRoute.size() - 1 ? true : false;}

        bool wpsEqual(const WayPointData &firstWP, const WayPointData &secondWP) {
            if(firstWP.altitude == secondWP.altitude && firstWP.N == secondWP.N && firstWP.E == secondWP.E && firstWP.speed == secondWP.speed) return true;
            return false;
        }
};

#endif