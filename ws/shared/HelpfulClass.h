#pragma once
#include "AMPCore.h"
//#include "ShareCore.h"

Eigen::Vector2d rotateVec2d(Eigen::Vector2d &vec, double angle);
std::vector<Eigen::Vector2d> minStartReorder(std::vector<Eigen::Vector2d> &av);
amp::Polygon minkowskiSum(const amp::Polygon a, const amp::Polygon b);
amp::Polygon translatePolygon(amp::Polygon &A, Eigen::Vector2d targ);
amp::Polygon rotatePolygon(amp::Polygon &poly, double angle);
amp::Polygon invertPolygon(amp::Polygon a);
void printPolygonVertices(amp::Polygon a);
bool intersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& q1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q2);
double pathDistance(amp::Path2D path);
bool collisionDetectedPoint(std::vector<amp::Obstacle2D> obstacles, Eigen::Vector2d position);
bool lineCollisionDection2D(std::vector<amp::Obstacle2D> o, Eigen::Vector2d v0, Eigen::Vector2d v1);
double randomDouble(double min, double max);
double vecAngle(Eigen::Vector2d v1, Eigen::Vector2d v2);
Eigen::Vector2d vectorXdToVector2d(const Eigen::VectorXd& v);
Eigen::Vector2d closestPointOnLine(const Eigen::Vector2d& A, const Eigen::Vector2d& B, const Eigen::Vector2d& P);
bool polyToPolyCollision(std::vector<amp::Obstacle2D> o, Eigen::Vector2d v0, Eigen::Vector2d v1, double r);
std::vector<amp::Obstacle2D> expandPolygons(std::vector<amp::Obstacle2D> obstacles, double expansionFactor);
amp::Obstacle2D expandBox(amp::Obstacle2D box, double dist);
std::vector<amp::Obstacle2D> expandBoxes(std::vector<amp::Obstacle2D> boxes, double dist);
std::vector<amp::Obstacle2D> expandPolygonsByEdge(std::vector<amp::Obstacle2D> obstacles, double expansionDist);