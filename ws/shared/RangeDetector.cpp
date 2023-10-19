#include "RangeDetector.h"

RangeDetector::RangeDetector() {
    range = 0;
    rays = 0;
}

RangeDetector::RangeDetector(double range, int rays) {
    this->range = range;
    this->rays = rays;
}



// Scan Function
void RangeDetector::scan(const std::vector<amp::Polygon>& obstacles, const Eigen::Vector2d& pos){
    //Reset the log for a new scan
    hitLog.clear();

    // For each ray in the sensor, check if it intersects with an obstacle
    for (int j = 0; j < obstacles.size(); j++) {

        Eigen::Vector2d ray;

        //Check if the ray intersects with any of the obstacles
        double minDist = INT32_MAX;
        Eigen::Vector2d minIntersectionPoint;

        //Check for collisions with obstacles
        for (int i = 0; i < rays; i++) {
            // Create a ray from the position of the robot
            ray = Eigen::Vector2d(cos(i*2*M_PI/rays), sin(i*2*M_PI/rays));

            for (int k = 0; k < obstacles[j].verticesCCW().size(); k++) {
                //If the ray intersects with an obstacle, add the obstacle to the list of obstacles in line of sight
                if (intersect(pos, pos + range*ray, obstacles[j].verticesCCW()[k], obstacles[j].verticesCCW()[(k+1)%obstacles[j].verticesCCW().size()])) {
                    //hitLog.push_back(makeCastHit(j, (lastHitPoint-pos).norm(), ray));
                    //std::cout << "At position: (" << pos.x() << ", " << pos.y() <<  ") ---> hit obstacle " << j << " at distance " << (lastHitPoint-pos).norm() << std::endl;

                    //Check if this collision was the shortest collision
                    if ((lastHitPoint-pos).norm() < minDist) {
                        minDist = (lastHitPoint-pos).norm();
                        minIntersectionPoint = lastHitPoint;
                    }
                }
            }
        }
        
        //If the ray intersected with an obstacle, add the obstacle to the list of obstacles in line of sight
        if (minDist != INT32_MAX) {
            hitLog.push_back(makeCastHit(j, minDist, (minIntersectionPoint - pos).normalized()));
            //std::cout << "Closest Encounter for Obj " << j << ": (" << pos.x() << ", " << pos.y() <<  ") at distance " << minDist << std::endl;
        }

        //Keep track of the object that was last hit until the next hit on the same object has a greater distance than the last ray
        //Once the closest distance has been found, add hit to the hitLog


    }
} 

// Scan Function
void RangeDetector::scanRaw(const std::vector<amp::Polygon>& obstacles, const Eigen::Vector2d& pos){
    //Reset the log for a new scan
    hitLog.clear();

    // For each ray in the sensor, check if it intersects with an obstacle
    for (int j = 0; j < obstacles.size(); j++) {

        Eigen::Vector2d ray;

        //Check for collisions with obstacles
        for (int i = 0; i < rays; i++) {

            //Check if the ray intersects with any of the obstacles
            double minDist = INT32_MAX;
            Eigen::Vector2d minIntersectionPoint;
            // Create a ray from the position of the robot
            ray = Eigen::Vector2d(cos(i*2*M_PI/rays), sin(i*2*M_PI/rays));

            for (int k = 0; k < obstacles[j].verticesCCW().size(); k++) {
                //If the ray intersects with an obstacle, add the obstacle to the list of obstacles in line of sight
                if (intersect(pos, pos + range*ray, obstacles[j].verticesCCW()[k], obstacles[j].verticesCCW()[(k+1)%obstacles[j].verticesCCW().size()])) {
                    //hitLog.push_back(makeCastHit(j, (lastHitPoint-pos).norm(), ray));
                    //std::cout << "At position: (" << pos.x() << ", " << pos.y() <<  ") ---> hit obstacle " << j << " at distance " << (lastHitPoint-pos).norm() << std::endl;

                    //Check if this collision was the shortest collision
                    if ((lastHitPoint-pos).norm() < minDist) {
                        minDist = (lastHitPoint-pos).norm();
                        minIntersectionPoint = lastHitPoint;
                    }
                }
            }

            //If the ray intersected with an obstacle, add the obstacle to the list of obstacles in line of sight
            if (minDist != INT32_MAX) {
                hitLog.push_back(makeCastHit(j, minDist, (minIntersectionPoint - pos).normalized()));
                //std::cout << "Closest Encounter for Obj " << j << ": (" << pos.x() << ", " << pos.y() <<  ") at distance " << minDist << std::endl;
            }
        }
    

        //Keep track of the object that was last hit until the next hit on the same object has a greater distance than the last ray
        //Once the closest distance has been found, add hit to the hitLog


    }
} 

bool RangeDetector::intersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& q1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q2){
    double x1 = p1.x();
    double y1 = p1.y();
    double x2 = q1.x();
    double y2 = q1.y();
    double x3 = p2.x();
    double y3 = p2.y();
    double x4 = q2.x();
    double y4 = q2.y();
    Eigen::Vector2d intersectionPoint;

    // Check if none of the lines are of length 0
    if ((x1 == x2 && y1 == y2) || (x3 == x4 && y3 == y4)) {
        return false;
    }
    double denom = ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
    // Lines are parallel
    if (denom == 0) {
        return false;
    }
    double ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3))/denom;
    double ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3))/denom;
    // is the intersection along the segments
    if (ua < 0 || ua > 1 || ub < 0 || ub > 1) {
        return false;
    }

    lastHitPoint = Eigen::Vector2d(x1 + ua * (x2 - x1), y1 + ua * (y2 - y1));
    return true; // is intersection
}

CastHit RangeDetector::makeCastHit(int obstacleIndex, double distance, Eigen::Vector2d direction) {
    CastHit hit;
    hit.obstacleIndex = obstacleIndex;
    hit.distance = distance;
    hit.direction = direction;
    return hit;
}

std::vector<CastHit> RangeDetector::getLog() {
    return hitLog;
    //return hitLogRaw;
}

//Takes the average direction of all hit points for each obstacle
std::vector<CastHit> RangeDetector::avgDirection(){
    if (hitLog.size() == 0){
        return hitLog;
    }
    //add upp all the direction vectors for or every CastHit object in the hitLog
    Eigen::Vector2d directionSum = Eigen::Vector2d(0.0, 0.0);
    std::vector<CastHit> newHitLog;
    std::vector<int> directionCount;
    std::vector<CastHit> avgDirection;
    int i = 1;
    int index = hitLog[0].obstacleIndex;
    double minDist = hitLog[0].distance;
    directionSum = directionSum + hitLog[0].direction;

    while (i < hitLog.size()){
        if (hitLog[i].obstacleIndex == index){
            directionSum = directionSum + hitLog[i].direction;
            if (hitLog[i].distance < minDist){
                minDist = hitLog[i].distance;
            }
            i++;
        } else{
            newHitLog.push_back(makeCastHit(index, minDist, directionSum.normalized()));
            directionSum = Eigen::Vector2d(0.0, 0.0);
            index = hitLog[i].obstacleIndex;
            minDist = hitLog[i].distance;
            directionSum = directionSum + hitLog[i].direction;
            i++;
        }
    }
    newHitLog.push_back(makeCastHit(index, minDist, directionSum.normalized()));

    //Print out the current hitlog
    // std::cout << "######## HitLog Before Averaging: ##########" << std::endl;
    // for (int i = 0; i < hitLog.size(); i++){
    //     std::cout << "HitLog: Index->" << hitLog[i].obstacleIndex << " Dist->" << hitLog[i].distance << " Direction->(" << hitLog[i].direction.x() << ", " << hitLog[i].direction.y() << ")" << std::endl;
    // }

    //copy the newHitLog into the hitLog
    hitLog = newHitLog;

        //Print out the current hitlog
    // std::cout << "######## HitLog After Averaging: ##########" << std::endl;
    // for (int i = 0; i < hitLog.size(); i++){
    //     std::cout << "HitLog: Index->" << hitLog[i].obstacleIndex << " Dist->" << hitLog[i].distance << " (" << hitLog[i].direction.x() << ", " << hitLog[i].direction.y() << ")" << std::endl;
    // }
    return newHitLog;
}