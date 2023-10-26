#include "PRM.h"

PRMAlgo::PRMAlgo(int n, double r, const amp::Problem2D& problem, Eigen::Vector2d q_start, Eigen::Vector2d q_goal, std::pair<double, double> xbounds, std::pair<double, double> ybounds) : amp::PRM(n, r, problem), n(n), r(r), problem_ptr(&problem), q_start(q_start), q_goal(q_goal), xbounds(xbounds), ybounds(ybounds) {}

void PRMAlgo::populateMap() {

    //Sample random node locations and add them to the node vector if they are collision free
    for (amp::Node i = 0; i < n; i++) {
        //Sample random node location
        double x = amp::randomDouble(xbounds.first, xbounds.second);
        double y = amp::randomDouble(ybounds.first, ybounds.second);
        Eigen::Vector2d q_rand = {x, y};

        //Check if the node is collision free
        if (collisionDetectedPoint(*problem_ptr, q_rand) == false) {
            PRMNode* node = new PRMNode;
            node->position = q_rand;
            nodeList.push_back(node);
        }
    } 

}

