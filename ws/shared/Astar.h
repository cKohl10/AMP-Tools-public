#pragma once

#include "ShareCore.h"
#include "AMPCore.h"
#include "hw/HW6.h"

struct AstarNode{
    AstarNode(amp::Node node, double edgeWeightFromStart, double heuristic, double edgeWeightFromParent, AstarNode* parent, std::vector<AstarNode*> children);
    amp::Node node;
    double edgeWeightFromStart;
    double heuristic;
    double edgeWeightFromParent;
    AstarNode* parent;
    std::vector<AstarNode*> children;
    bool processed = false;
};

class MyAStarAlgo : public amp::AStar {
    public:

        //Takes in a problem and a heuristic and returns a GraphSearchResult
        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override;
            
        //Print out the final path found
        void printPath(AstarNode* node);

};