#pragma once

#include "AMPCore.h"

struct CastHit {
    int obstacleIndex;
    double distance;
    Eigen::Vector2d direction;
};

class RangeDetector{
    public:
        RangeDetector();
        RangeDetector(double range, int rays);

        void scan(const std::vector<amp::Polygon>& obstacles, const Eigen::Vector2d& pos);
        void scanRaw(const std::vector<amp::Polygon>& obstacles, const Eigen::Vector2d& pos);

        CastHit makeCastHit(int obstacleIndex, double distance, Eigen::Vector2d direction);
        std::vector<CastHit> getLog();
        std::vector<CastHit> avgDirection();
    
    private: 
        //############# HYPER PARAMETERS #############
        double range; //Range of the sensor
        int rays; //Number of rays the sensor has
        //############################################

        std::vector<CastHit> hitLog; //A vector of all the closest distance hits for each obstacle
        Eigen::Vector2d lastHitPoint; //The last point that was hit by the sensor, do not call anywhere

        bool intersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& q1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q2);
};