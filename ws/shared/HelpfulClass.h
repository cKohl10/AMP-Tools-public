#pragma once
#include "AMPCore.h"

Eigen::Vector2d rotateVec2d(Eigen::Vector2d &vec, double angle);
std::vector<Eigen::Vector2d> minStartReorder(std::vector<Eigen::Vector2d> &av);
amp::Polygon minkowskiSum(const amp::Polygon a, const amp::Polygon b);
amp::Polygon translatePolygon(amp::Polygon &A, Eigen::Vector2d targ);
amp::Polygon rotatePolygon(amp::Polygon &poly, double angle);
amp::Polygon invertPolygon(amp::Polygon a);
void printPolygonVertices(amp::Polygon a);
bool intersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& q1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q2);

double vecAngle(Eigen::Vector2d v1, Eigen::Vector2d v2);