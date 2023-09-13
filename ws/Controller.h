#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"
#include "helperFunc.h"

//Controller class is used to move the bug, track obstacles, and not allow the bug to move where there are collisions

class Controller{
    public:
        Controller(amp::Problem2D p);
        Eigen::Vector2d step(Eigen::Vector2d q_last, double stepSize);

        void changeFollowTarget(int obstaceleIndex);
        void printtargetQueue();
        void setFollowTarget(int obstacleIndex);
        void clearTargetQueue();
        Eigen::Vector2d getCurrentTarget();


    private:
        amp::Problem2D problem;
        std::vector<Eigen::Vector2d> targetQueue;
        int targetObstacleIndex;

};