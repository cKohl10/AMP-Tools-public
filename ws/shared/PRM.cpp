#include "PRM.h"

PointCollisionChecker2D::PointCollisionChecker2D(const Eigen::VectorXd& lower_bounds, const Eigen::VectorXd& upper_bounds,  const amp::Environment2D* environment) : amp::ConfigurationSpace(lower_bounds, upper_bounds) {
    std::cout << "PointCollisionChecker2D constructor called" << std::endl;
    this->m_environment = environment;
}

bool PointCollisionChecker2D::inCollision(const Eigen::VectorXd& state) const{
    return false;
}

amp::Path GenericPRM::planxd(const Eigen::VectorXd& init_state, const Eigen::VectorXd& goal_state, PointCollisionChecker2D* collision_checker){
    std::cout << "Called GenericPRM::plan" << std::endl;

    //Make Path Object
    amp::Path path_nd;

    //Add the first node to the vector of nodes
    std::map<amp::Node, Eigen::VectorXd> node_map;

    //Add the initial state and goal state to the node map
    node_map[0] = init_state;
    node_map[1] = goal_state;

    //Sample n times and add the nodes to the node map
    for (amp::Node i = 2; i < n; i++) {

        //Create a new node the same dimension as the init_state
        Eigen::VectorXd q_rand(init_state.size());

        //Sample random node location
        for (int j = 0; j < bounds.size(); j++) {   
            q_rand[j] = randomDouble(bounds[j][0], bounds[j][1]);
        }

        //Check if the node is collision free
        if (collision_checker->inCollision(q_rand) == false) {
            node_map[i] = q_rand;
        }
    }

    //print the node map
    for (auto const& x : node_map)
    {
        std::cout << x.first  // string (key)
                  << ':'
                  << x.second.transpose() << std::endl;
    }

    return path_nd;
}

PRMAlgo2D::PRMAlgo2D(){
    //Generic Constructor
}

//Initialize with the Cspace bounds
PRMAlgo2D::PRMAlgo2D(Eigen::Vector2d xbounds, Eigen::Vector2d ybounds){
    //DEBUGGING
    std::cout << "PRMAlgo2D Constructed with xbounds (" << xbounds[0] << ", " << xbounds[1] << ") and ybounds (" << ybounds[0] << ", " << ybounds[1] << ")" << std::endl;   
    this->bounds.push_back(xbounds);
    this->bounds.push_back(ybounds);
}

//Initialize with the Cspace bounds
PRMAlgo2D::PRMAlgo2D(Eigen::Vector2d xbounds, Eigen::Vector2d ybounds, int n, double r){
    //DEBUGGING
    std::cout << "PRMAlgo2D Constructed with xbounds (" << xbounds[0] << ", " << xbounds[1] << ") and ybounds (" << ybounds[0] << ", " << ybounds[1] << ")" << std::endl;   
    this->bounds.push_back(xbounds);
    this->bounds.push_back(ybounds);
    this->n = n;
    this->r = r;
}


amp::Path2D PRMAlgo2D::plan(const amp::Problem2D& problem){

    //Make Cspace
    PointCollisionChecker2D* cspace_ptr = new PointCollisionChecker2D(bounds[0], bounds[1], &problem);

    //Make unique pointer to the Cspace
    //std::unique_ptr<amp::ConfigurationSpace> cspace_ptr = std::make_unique<PointCollisionChecker2D>(xbounds, ybounds, &problem);

    //Convert the problem start and goal states to xd matrices
    Eigen::VectorXd init_state(2);
    Eigen::VectorXd goal_state(2);
    init_state << problem.q_init[0], problem.q_init[1];
    goal_state << problem.q_goal[0], problem.q_goal[1];

    //Call generic planner
    amp::Path path_nd = planxd(init_state, goal_state, cspace_ptr);

    // Convert the ND path to a 2D path and return it...
    return amp::Path2D();
}

// void PRMAlgo2D::populateMap() {

//     //Sample random node locations and add them to the node vector if they are collision free
//     for (amp::Node i = 0; i < n; i++) {
//         //Sample random node location
//         double x = randomDouble(xbounds.first, xbounds.second);
//         double y = randomDouble(ybounds.first, ybounds.second);
//         Eigen::Vector2d q_rand = {x, y};

//         //Check if the node is collision free
//         if (collisionDetectedPoint(problem_ptr->obstacles, q_rand) == false) {
//             PRMNode* node = new PRMNode;
//             node->position = q_rand;
//             nodeList.push_back(node);
//         }
//     } 

// }

