#pragma once

//#include "ShareCore.h"
#include "AMPCore.h"
//#include "hw/HW6.h"

struct AstarNode{
    AstarNode(amp::Node node, double edgeWeightFromStart, double heuristic, double edgeWeightFromParent, std::shared_ptr<AstarNode> parent, std::vector<std::shared_ptr<AstarNode>> children);
    amp::Node node;
    double edgeWeightFromStart;
    double heuristic;
    double edgeWeightFromParent;
    std::shared_ptr<AstarNode> parent;
    std::vector<std::shared_ptr<AstarNode>> children;
    bool processed = false;

    void print();
};

class MyAStarAlgo : public amp::AStar {
    public:

        //Takes in a problem and a heuristic and returns a GraphSearchResult
        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override;

        /// @brief Takes the results and turns to a path
        /// @param problem 
        /// @param heuristic 
        /// @param node_map 
        /// @return 
        amp::Path searchPath(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic, std::map<amp::Node, Eigen::VectorXd> node_map);
            
        //Print out the final path found
        void printPath(std::shared_ptr<AstarNode> node);

};