#pragma once

#include "ShareCore.h"
#include "AMPCore.h"
#include "hw/HW7.h"

struct PRMNode{
    Eigen::Vector2d position;
    std::vector<PRMNode*> neighbors;
};

class PRMAlgo : public amp::PRM {
    public:

        PRMAlgo(int n, double r, const amp::Problem2D& problem, Eigen::Vector2d q_start, Eigen::Vector2d q_goal, std::pair<double, double> xbounds, std::pair<double, double> ybounds);

        //Takes in a problem and a heuristic and returns a GraphSearchResult
        //virtual amp::Path2D plan(const amp::Problem2D& problem) override;
            
        //Print out the final path found
        //void printPath(AstarNode* node);

        void populateMap();

    private:
        int n;
        double r;
        amp::Problem2D* problem_ptr;
        Eigen::Vector2d q_start;
        Eigen::Vector2d q_goal;
        std::pair<double, double> xbounds
        std::pair<double, double> ybounds

        std::vector<PRMNode*> nodeList;

};