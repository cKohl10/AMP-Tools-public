#include "CellGraph.h"

CellGraph::CellGraph(std::pair<int, int> start, std::pair<int, int> goal, const amp::GridCSpace2D& cspace){
    //Initialize the neighbor order
    neighborOrder.push_back(std::make_pair(0, 1));
    neighborOrder.push_back(std::make_pair(1, 1));
    neighborOrder.push_back(std::make_pair(1, 0));
    neighborOrder.push_back(std::make_pair(1, -1));
    neighborOrder.push_back(std::make_pair(0, -1));
    neighborOrder.push_back(std::make_pair(-1, -1));
    neighborOrder.push_back(std::make_pair(-1, 0));
    neighborOrder.push_back(std::make_pair(-1, 1));

    //Initialize the edge neighbor order
    edgeNeighborOrder.push_back(std::make_pair(0, 1));
    edgeNeighborOrder.push_back(std::make_pair(1, 0));
    edgeNeighborOrder.push_back(std::make_pair(0, -1));
    edgeNeighborOrder.push_back(std::make_pair(-1, 0));

    //Create the start and goal nodes
    startNode = new Node;
    startNode->cellPos = start;
    startNode->distFromStart = 0;
    goalNode = new Node;
    goalNode->cellPos = goal;
    goalNode->distFromStart = -1;

    //Add the start and goal nodes to the graph
    nodes.push_back(*startNode);

    //Create the empty cspace as a copy from cspace
    //std::unique_ptr<amp::GridCSpace2D> = std::make_unique<MyGridCSpace>(cspace.size().first, cspace.size().second, cspace.x0Bounds().first, cspace.x0Bounds().second, cspace.x1Bounds().first, cspace.x1Bounds().second);

    //Populating the graph in regards to the start node and environment bounds
    std::cout << "Populating graph..." << std::endl;

}
