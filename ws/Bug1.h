#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"
#include "helperFunc.h"
#include "Controller.h"

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class Bug1 : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) const override;

        // Add any other methods here...
    
    private:
        // Add any member variables here...

        //Stores all the points on the boundary of the path
        std::vector<Eigen::Vector2d> obstaclePath;

        //Stores the quickest path to q_L from the obstacle path object
        std::vector<Eigen::Vector2d> shortestObstaclePath;
};