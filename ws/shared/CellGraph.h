#pragma once

#include "AMPCore.h"
#include "ShareCore.h"

struct Node{
    std::pair<int, int> cellPos;
    std::vector<Node*> neighbors;
    int distFromStart;
};

//This class is used to create a graph of cells that are connected to each other. Used in wavefront algorithm
class CellGraph{
    public:
        CellGraph(std::pair<int, int> start, std::pair<int, int> goal, const amp::GridCSpace2D& cspace);
        //Node addNode(std::pair<int, int> cellPos, int distFromStart);
        //void populateGraph();

    private:
        std::vector<Node> nodes;
        Node* startNode;
        Node* goalNode;
        std::vector<std::pair<int, int>> edgeNeighborOrder;
        std::vector<std::pair<int, int>> neighborOrder;


};