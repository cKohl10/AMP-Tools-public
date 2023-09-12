#pragma once

#include "Bug1.h"
#include "hw/HW2.h"
#include "helperFunc.h"

// This controller is intended to be used for any bug algorithm
// It will take in a direction vector, and step the bug in that direction, checking for collisions, and adjusting 
// based off a specified distance away from the obstacle boundry.

class Controller {
    public:
        Eigen::Vector2d step(const amp::Problem2D& problem, Eigen::Vector2d q_last, std::vector<Eigen::Vector2d> q_targetList, int targDex, double stepSize, double radius) const;

    private:

};