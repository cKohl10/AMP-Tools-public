#pragma once

#include "AMPCore.h"
#include "LinkMan.h"
#include "Eigen/Geometry"

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
        CSpace(const amp::LinkManipulator2D& robot, const amp::Environment2D& enviroment, std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max);

        bool inCollision(double x0, double x1) const;

        std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const;

    private:
        bool collisionDetected(Eigen::Vector2d position) const;
        bool subdivide(Eigen::Vector2d b1, Eigen::Vector2d b2, int counter) const;
        bool intersect(Eigen::Vector2d p1, Eigen::Vector2d q1, Eigen::Vector2d p2, Eigen::Vector2d q2) const;
        const amp::LinkManipulator2D *m_robot;
        amp::Environment2D m_environment;
};

