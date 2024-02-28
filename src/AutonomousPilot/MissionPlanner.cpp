#include "MissionPlanner.hpp"

MissionPlanner::MissionPlanner(std::string &missionWayPointsStr) : currentWP(1) {
    std::vector<std::string> lines;
    int delimIndex = missionWayPointsStr.find_first_of("\n");
    
    while(delimIndex != std::string::npos) {
        // insert all lines (except last line -> there is no \n in eos)
        lines.push_back(missionWayPointsStr.substr(0, delimIndex));
        missionWayPointsStr = missionWayPointsStr.substr(delimIndex + 1);
        delimIndex = missionWayPointsStr.find_first_of("\n");
    }
    // insert last line
    lines.push_back(missionWayPointsStr);
    
    // remove title line
    lines.erase(lines.begin());
    
    // parse each line and create WayPointData vector
    for(std::string line : lines) {
        WayPointData wp;
        wp.N = stof(line.substr(0, line.find_first_of(", ")));
        line = line.substr(line.find_first_of(", ") + 2);
        wp.E = stof(line.substr(0, line.find_first_of(", ")));
        line = line.substr(line.find_first_of(", ") + 2);
        wp.altitude = stod(line.substr(0, line.find_first_of(", ")));
        line = line.substr(line.find_first_of(", ") + 2);
        wp.speed = stof(line);
        
        this->missionRoute.push_back(wp);
    }

    // flight route need to be in size of atleast 3, (takeoff, first point, landing)
    if(this->missionRoute.size() < 3) this->currentWP = -1;
}

bool MissionPlanner::isInWPZone(const WayPointData &sensorsData) {
    // Convert latitude and longitude from degrees to radians
    double lat1 = degreesToRadians(sensorsData.N);
    double lon1 = degreesToRadians(sensorsData.E);
    double alt1 = sensorsData.altitude;

    double lat2 = degreesToRadians(this->missionRoute[currentWP].N);
    double lon2 = degreesToRadians(this->missionRoute[currentWP].E);
    double alt2 = this->missionRoute[currentWP].altitude;

    // Calculate reduced latitudes
    double tanU1 = (1 - f) * tan(lat1);
    double tanU2 = (1 - f) * tan(lat2);
    double cosU1 = 1 / sqrt(1 + tanU1 * tanU1);
    double cosU2 = 1 / sqrt(1 + tanU2 * tanU2);
    double sinU1 = tanU1 * cosU1;
    double sinU2 = tanU2 * cosU2;

    // Initial values
    double lambda = lon2 - lon1;
    double lambdaP = 2 * M_PI;
    double iterLimit = 100; // Maximum number of iterations
    double cosAlpha, sinAlpha;
    double sigma, cosSigma, sinSigma, cos2SigmaM;
    double u1 = atan(tanU1);
    double u2 = atan(tanU2);

    while (fabs(lambda - lambdaP) > 1e-12 && iterLimit > 0) {
        double sinLambda = sin(lambda);
        double cosLambda = cos(lambda);
        sinAlpha = sqrt((cosU2 * sinLambda) * (cosU2 * sinLambda) + 
                        (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda) * 
                        (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda));
        cosAlpha = sinU1 * sinU2 + cosU1 * cosU2 * cosLambda;
        sigma = atan2(sinAlpha, cosAlpha);
        sinSigma = sin(sigma);
        cosSigma = cos(sigma);
        cos2SigmaM = cosU1 * cosU2 * sinLambda / sinSigma;

        // Update values for the next iteration
        double C = f / 16 * cosU1 * cosU2 * (4 + f * (4 - 3 * cosAlpha * cosAlpha));
        lambdaP = lambda;
        lambda = lon2 - lon1 + (1 - C) * f * sinAlpha * (sigma + C * sinSigma * (cos2SigmaM + C * cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM)));
        iterLimit--;
    }

    if (iterLimit == 0) {
        // Formula did not converge
        return false;
    }

    // Calculate final values
    double uSq = cosAlpha * cosAlpha * (a * a - b * b) / (b * b);
    double A = 1 + uSq / 16384 * (4096 + uSq * (-768 + uSq * (320 - 175 * uSq)));
    double B = uSq / 1024 * (256 + uSq * (-128 + uSq * (74 - 47 * uSq)));
    double deltaSigma = B * sinSigma * (cos2SigmaM + B / 4 * (cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM) - B / 6 * cos2SigmaM * (-3 + 4 * sinSigma * sinSigma) * (-3 + 4 * cos2SigmaM * cos2SigmaM)));
    
    // Calculate the distance
    double s = b * A * (sigma - deltaSigma);

    // Add altitude difference
    double altitudeDifference = std::fabs(alt1 - alt2);

    return sqrt(s * s + altitudeDifference * altitudeDifference) < this->inZoneDistanceCheck ? true : false;
}

double MissionPlanner::flatDistance(const WayPointData &sensorsData, const int wpIndex) {
    // Convert latitude and longitude from degrees to radians
    double lat1 = degreesToRadians(sensorsData.N);
    double lon1 = degreesToRadians(sensorsData.E);

    double lat2 = degreesToRadians(this->missionRoute[currentWP + wpIndex].N);
    double lon2 = degreesToRadians(this->missionRoute[currentWP + wpIndex].E);

    // Calculate reduced latitudes
    double tanU1 = (1 - f) * tan(lat1);
    double tanU2 = (1 - f) * tan(lat2);
    double cosU1 = 1 / sqrt(1 + tanU1 * tanU1);
    double cosU2 = 1 / sqrt(1 + tanU2 * tanU2);
    double sinU1 = tanU1 * cosU1;
    double sinU2 = tanU2 * cosU2;

    // Initial values
    double lambda = lon2 - lon1;
    double lambdaP = 2 * M_PI;
    double iterLimit = 100;
    double cosAlpha, sinAlpha;
    double sigma, cosSigma, sinSigma, cos2SigmaM;
    double u1 = atan(tanU1);
    double u2 = atan(tanU2);

    while (fabs(lambda - lambdaP) > 1e-12 && iterLimit > 0) {
        double sinLambda = sin(lambda);
        double cosLambda = cos(lambda);
        sinAlpha = sqrt((cosU2 * sinLambda) * (cosU2 * sinLambda) + 
                        (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda) * 
                        (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda));
        cosAlpha = sinU1 * sinU2 + cosU1 * cosU2 * cosLambda;
        sigma = atan2(sinAlpha, cosAlpha);
        sinSigma = sin(sigma);
        cosSigma = cos(sigma);
        cos2SigmaM = cosU1 * cosU2 * sinLambda / sinSigma;

        // Update values for the next iteration
        double C = f / 16 * cosU1 * cosU2 * (4 + f * (4 - 3 * cosAlpha * cosAlpha));
        lambdaP = lambda;
        lambda = lon2 - lon1 + (1 - C) * f * sinAlpha * (sigma + C * sinSigma * (cos2SigmaM + C * cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM)));
        iterLimit--;
    }

    if (iterLimit == 0) {
        // Formula did not converge
        return false;
    }

    // Calculate final values
    double uSq = cosAlpha * cosAlpha * (a * a - b * b) / (b * b);
    double A = 1 + uSq / 16384 * (4096 + uSq * (-768 + uSq * (320 - 175 * uSq)));
    double B = uSq / 1024 * (256 + uSq * (-128 + uSq * (74 - 47 * uSq)));
    double deltaSigma = B * sinSigma * (cos2SigmaM + B / 4 * (cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM) - B / 6 * cos2SigmaM * (-3 + 4 * sinSigma * sinSigma) * (-3 + 4 * cos2SigmaM * cos2SigmaM)));
    
    // Calculate the distance
    double s = b * A * (sigma - deltaSigma);

    return sqrt(pow(s, 2));
}

double MissionPlanner::calculateInitialBearingAndRequiredYaw(const WayPointData &sensorData, const float &yawAngle) {
    // Convert latitude and longitude from degrees to radians    
    double lat1 = degreesToRadians(sensorData.N);
    double lon1 = degreesToRadians(sensorData.E);

    double lat2 = degreesToRadians(this->missionRoute[currentWP].N);
    double lon2 = degreesToRadians(this->missionRoute[currentWP].E);

    // Calculate reduced latitudes and other intermediate values
    double tanU1 = (1 - f) * tan(lat1);
    double tanU2 = (1 - f) * tan(lat2);
    double cosU1 = 1 / sqrt(1 + tanU1 * tanU1);
    double cosU2 = 1 / sqrt(1 + tanU2 * tanU2);
    double sinU1 = tanU1 * cosU1;
    double sinU2 = tanU2 * cosU2;

    // Calculate difference in longitudes
    double lambda = lon2 - lon1;

    // Calculate initial bearing (θ) using Vincenty formula
    double initialBearing = atan2(sin(lambda) * cosU2, cosU1 * sinU2 - sinU1 * cosU2 * cos(lambda));

    // Convert from radians to degrees
    initialBearing = initialBearing * 180.0 / M_PI;

    double requiredYaw = initialBearing - yawAngle;

    // Normalize the yaw angle to the range -180 to 180 degrees
    while (requiredYaw > 180.0) {
        requiredYaw -= 360.0;
    }
    while (requiredYaw < -180.0) {
        requiredYaw += 360.0;
    }

    return requiredYaw;
}