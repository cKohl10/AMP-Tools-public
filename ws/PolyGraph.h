#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"
#include "helperFunc.h"

class PolyGraph{
    public:
        std::vector<std::vector<amp::Obstacle2D>> obstacleGroups;
        std::vector<amp::Obstacle2D> obstacles;
        std::vector<std::vector<Eigen::Vector2d>> vertices;
        std::vector<bool> visited;

        PolyGraph(amp::Problem2D problem);
        void sort();
        std::vector<amp::Obstacle2D> DFS(int i, std::vector<amp::Obstacle2D> group);
        void findVertices();
        void findIntersectionVertices();
        void commonContour();

    private:


};