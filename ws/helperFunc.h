#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"
#include "Eigen/Geometry"

Eigen::Vector2d directionVec(Eigen::Vector2d fromPoint, Eigen::Vector2d toPoint);
bool goalReached(Eigen::Vector2d currentPoint, Eigen::Vector2d q_goal, double goalReachedError);
int collisionDetected(std::vector<amp::Obstacle2D> obstacles, Eigen::Vector2d position);
std::vector<Eigen::Vector2d> closestPointOnObstacle(Eigen::Vector2d q_last, Eigen::Vector2d q_next, amp::Obstacle2D obstacle);
double pathDistance(amp::Path2D path);
double vertVecDistance(std::vector<Eigen::Vector2d> path);
std::vector<Eigen::Vector2d> findShortestPath(std::vector<Eigen::Vector2d> obstaclePath, Eigen::Vector2d q_goal);

// Enviroment Fixing Functions
amp::Problem2D expandPolygons(amp::Problem2D problem, double expansionFactor);
amp::Problem2D fixOverlappingPolygons(amp::Problem2D problem);
bool polygonOverlap(amp::Obstacle2D obstacle1, amp::Obstacle2D obstacle2);