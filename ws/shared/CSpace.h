#pragma once

#include "AMPCore.h"
#include "LinkMan.h"

class CSpace : public amp::GridCSpace2D {
    public:

        /// @brief Constructor for robot in c-space
        /// @param robot 
        /// @param enviroment 
        /// @param x0_cells 
        /// @param x1_cells 
        /// @param x0_min 
        /// @param x0_max 
        /// @param x1_min 
        /// @param x1_max 
        CSpace(LinkMan robot, amp::Environment2D enviroment, std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max);

        bool inCollision(double x0, double x1) const;
        bool collisionDetected(Eigen::Vector2d position);

    private:
        LinkMan m_robot;
        amp::Environment2D m_enviroment;
};