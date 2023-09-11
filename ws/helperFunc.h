#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

Eigen::Vector2d directionVec(Eigen::Vector2d fromPoint, Eigen::Vector2d toPoint);
bool goalReached(Eigen::Vector2d currentPoint, Eigen::Vector2d q_goal, double goalReachedError);
int collisionDetected(std::vector<amp::Obstacle2D> obstacles, Eigen::Vector2d position);
Eigen::Vector2d closestPointOnObstacle(Eigen::Vector2d q_last, Eigen::Vector2d q_next, amp::Obstacle2D obstacle);