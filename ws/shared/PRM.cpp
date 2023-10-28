#include "PRM.h"

PointCollisionChecker2D::PointCollisionChecker2D(const Eigen::VectorXd& lower_bounds, const Eigen::VectorXd& upper_bounds,  const amp::Environment2D* environment) : amp::ConfigurationSpace(lower_bounds, upper_bounds) {
    std::cout << "PointCollisionChecker2D constructor called" << std::endl;
    this->m_environment = environment;
}

bool PointCollisionChecker2D::inCollision(const Eigen::VectorXd& state) const{

    // ######## NOTE: This is only currently set up for 2D ########
    Eigen::Vector2d state2D = {state[0], state[1]};
    return collisionDetectedPoint(m_environment->obstacles, state2D);
}

const amp::Environment2D* PointCollisionChecker2D::getEnvironment() { return m_environment; }

amp::Path GenericPRM::planxd(const Eigen::VectorXd& init_state, const Eigen::VectorXd& goal_state, PointCollisionChecker2D* collision_checker){
    std::cout << "Called GenericPRM::plan" << std::endl;

    //Make Path Object
    amp::Path path_nd;

    //Add the initial state and goal state to the node map
    node_map[0] = init_state;
    node_map[1] = goal_state;

    //Sample n times and add the nodes to the node map
    sampleMap(node_map, collision_checker);

    //print the node map
    // for (auto const& x : node_map)
    // {
    //     std::cout << x.first  // string (key)
    //               << ':'
    //               << x.second.transpose() << std::endl;
    // }

    //Connect the neighbors of each node
    for (auto const& x : node_map)
    {
        //Get the node number
        amp::Node node_num = x.first;

        //Get the node position
        Eigen::VectorXd node_pos = x.second;

        //Check every other node to see if it is within the neighborhood
        for (auto const& y : node_map)
        {
            //Get the node number
            amp::Node neighbor_num = y.first;

            //Get the node position
            Eigen::VectorXd neighbor_pos = y.second;

            //Check if the node is within the neighborhood
            if ((node_pos - neighbor_pos).norm() < r && node_num != neighbor_num) {

                //Check if there is a collision in the path between the two nodes
                //########## NOTE: CURRENTLY ONLY IMPLEMENTED FOR 2D ##############
                //Convert the node positions to 2D vectors
                Eigen::Vector2d node_pos2D = {node_pos[0], node_pos[1]};
                Eigen::Vector2d neighbor_pos2D = {neighbor_pos[0], neighbor_pos[1]};
                if (lineCollisionDection2D(collision_checker->getEnvironment()->obstacles, node_pos2D, neighbor_pos2D) == true){
                    continue;
                }
                //##################################################################

                //Add the edge to the graph
                graph.connect(node_num, neighbor_num, (node_pos - neighbor_pos).norm());
            }
        }

    }

    //graph.print();

    //Find the shortest path
    std::cout << "Making Shortest Path Problem..." << std::endl;
    amp::ShortestPathProblem searchProblem;
    searchProblem.graph = std::make_shared<amp::Graph<double>>(graph);
    searchProblem.init_node = 0;
    searchProblem.goal_node = 1;

    //Traverse the graph
    std::cout << "Finding Shortest Path..." << std::endl;
    MyAStarAlgo astar;
    path_nd = astar.searchPath(searchProblem, heuristic, node_map);
    
    return path_nd;
}

void GenericPRM::sampleMap(std::map<amp::Node, Eigen::VectorXd>& node_map, PointCollisionChecker2D* collision_checker){
    std::cout << "Sampling Each Node..." << std::endl;

    int node_size = node_map[0].size();

    Eigen::VectorXd goal_pos = node_map[1];

    //Set the heuristics for goal and start
    heuristic.heuristic_values[0] = 0;
    heuristic.heuristic_values[1] = 0;

    //Sample n times and add the nodes to the node map
    for (amp::Node i = 2; i < n; i++) {

        //Create a new node the same dimension as the init_state
        Eigen::VectorXd q_rand(node_size);

        //Sample random node location
        for (int j = 0; j < bounds.size(); j++) {   
            q_rand[j] = randomDouble(bounds[j][0], bounds[j][1]);
        }

        //Check if the node is collision free
        if (collision_checker->inCollision(q_rand) == true) {
            continue;
        }

        //Add node to map
        node_map[i] = q_rand;

        //Determine its heuristic value
        heuristic.heuristic_values[i] = (q_rand-goal_pos).norm();

    }

    //print out heuristic map
    // for (auto const& x : heuristic.heuristic_values){
    //     std::cout << "Node " << x.first << " has heuristic value " << x.second << std::endl;
    // }
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

    //Recreate node map to have 2d vectors
    std::cout << "Converting General Map to 2D Map..." << std::endl;
    for (auto const& x : node_map)
    {
        //Get the node number
        amp::Node node_num = x.first;

        //Get the node position
        Eigen::VectorXd node_pos = x.second;

        //Create a 2d vector
        Eigen::Vector2d node_pos_2d;
        node_pos_2d << node_pos[0], node_pos[1];

        //Add the node to the 2d node map
        node_map2D[node_num] = node_pos_2d;
    }

    //Check the graph made by the generic planner
    amp::Visualizer::makeFigure(problem, graph, node_map2D);

    //Make 2D path object
    amp::Path2D path;

    // Convert the ND path to a 2D path and return it...
    std::cout << "Converting General Path to 2D Path..." << std::endl;
    for (int i = 0; i < path_nd.waypoints.size(); i++) {
        Eigen::Vector2d waypoint;
        waypoint << path_nd.waypoints[i][0], path_nd.waypoints[i][1];
        path.waypoints.push_back(waypoint);
    }

    //Return the path
    return path;
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

