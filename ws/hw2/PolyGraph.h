#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"
#include "helperFunc.h"

class PolyGraph{
    public:
        std::vector<std::vector<amp::Obstacle2D>> obstacleGroups; //Stores the obstacles in groups of overlapping polygons
        std::vector<amp::Obstacle2D> obstacles; //Stores all the obstacles
        std::vector<std::vector<Eigen::Vector2d>> vertices; //Stores all the vertices of the overlapping polygons in groups
        std::vector<std::vector<Eigen::Vector2d>> newObstacleVerticesSet; //Stores the new vertices of the obstacles after the common contour is found
        std::vector<bool> visited;

        PolyGraph(amp::Problem2D problem);
        void sort();
        std::vector<amp::Obstacle2D> DFS(int i, std::vector<amp::Obstacle2D> group);
        void findVertices();
        void findIntersectionVertices();
        void commonContour();

    private:


};