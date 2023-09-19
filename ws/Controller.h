#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"
#include "helperFunc.h"

//Controller class is used to move the bug, track obstacles, and not allow the bug to move where there are collisions

class Controller{
    public:
        Controller(amp::Problem2D p,  bool rightTurning = true);
        Eigen::Vector2d step(Eigen::Vector2d q_last, double stepSize);

        //Target object and queue
        void changeFollowTarget(int obstaceleIndex);
        void printtargetQueue();
        void setFollowTarget(int obstacleIndex);
        void clearTargetQueue();
        void nextTarget();
        Eigen::Vector2d getCurrentTarget();
        std::vector<Eigen::Vector2d> getTargetQueue();
        int getCurrentFollowObstacle();

        //Path tracking
        void logPointInObstaclePath(Eigen::Vector2d q);
        std::vector<Eigen::Vector2d> getObstaclePathTaken();

        //Reset for leaving the obstacle
        void clearBug();

        //Bug 2
        bool checkMLineIntersection(Eigen::ParametrizedLine<double, 2> mLine, Eigen::Vector2d q_goal, Eigen::Vector2d q);

        double distTraveled;

    private:
        amp::Problem2D problem;
        std::vector<Eigen::Vector2d> targetQueue;
        std::vector<Eigen::Vector2d> obstaclePathTaken;
        int targetObstacleIndex;
        bool rightSideOfMLine;
        bool leftTurningBug;

};