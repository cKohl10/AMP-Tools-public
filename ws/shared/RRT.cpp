#include "RRT.h"

RRTAlgo2D::RRTAlgo2D(double r, double p_goal, int n, double epsilon, Eigen::Vector2d xbounds, Eigen::Vector2d ybounds){
    this->r = r;
    this->p_goal = p_goal;
    this->n = n;
    this->epsilon = epsilon;
    this->bounds = {xbounds, ybounds};
}

amp::Path RRTAlgo2D::planxd(const amp::Problem2D& problem){

    //Debug
    std::cout << "Running RRT Algorithm..." << std::endl;

    //Make Path Object
    amp::Path path_nd;
    int node_size = problem.q_init.size();

    //Initialize the root node
    amp::Node root_node = 0;
    Eigen::VectorXd q_root(node_size);
    q_root << problem.q_init[0], problem.q_init[1];
    node_map[root_node] = q_root;

    //Initialize the goal node
    amp::Node goal_node = 1;
    Eigen::VectorXd q_goal(node_size);
    q_goal << problem.q_goal[0], problem.q_goal[1];

    //Inialize node counter
    amp::Node node_counter = 2;

    //Start search loop
    while (node_counter < n){

        //Create a new node the same dimension as the init_state
        Eigen::VectorXd q_rand(node_size);

        //Sample random node with probability p_goal
        if (randomDouble(0, 1) > p_goal){
            std::cout << "Sampling Random Node..." << std::endl;
            //Sample random node location
            for (int j = 0; j < bounds.size(); j++) {   
                q_rand[j] = randomDouble(bounds[j][0], bounds[j][1]);
            }
        } else{
            std::cout << "Sampling Goal Node..." << std::endl;
            //Sample goal node location
            q_rand = q_goal;
        }

        //Find the nearest node to the random node
        std::cout << "Finding Nearest Node..." << std::endl;
        amp::Node nearest_node = nearestNeighbor(q_rand);
        Eigen::VectorXd q_near = node_map[nearest_node];

        //Create a new node in the direction of the random node
        Eigen::VectorXd q_new = q_near + (q_rand - q_near).normalized() * r;

        //Check if the new node is in collision
        // NOTE: This is a 2D collision checker
        if (lineCollisionDection2D(problem.obstacles, vectorXdToVector2d(q_near), vectorXdToVector2d(q_new)) == false){
            
            //Add the new node to the node map
            node_map[node_counter] = q_new;

            //Add the new node to the graph
            graph.connect(nearest_node, node_counter, r);

            //Check if the new node is within the termination radius
            if ((q_new - q_goal).norm() < epsilon){

                std::cout << "Goal Reached!" << std::endl;

                //Add the goal node to the node map
                node_map[goal_node] = q_goal;

                //Add the goal node to the graph
                graph.connect(node_counter, goal_node, (q_new-q_goal).norm());

                //Build the path
                amp::Node curr_node = goal_node;
                while (curr_node != 0){
                    path_nd.waypoints.push_back(node_map[curr_node]);
                    curr_node = graph.parents(curr_node)[0];
                }
                path_nd.waypoints.push_back(node_map[curr_node]);

                //Return the path
                return path_nd;
            }
        }

        node_counter++;
        std::cout << "Node " << node_counter << "/" << n << " Created" << std::endl;

    }
    return path_nd;
}

amp::Path2D RRTAlgo2D::plan(const amp::Problem2D& problem){
    amp::Path path_xd = planxd(problem);

    //Convert the path to 2D
    amp::Path2D path_2d;
    for (auto& x : path_xd.waypoints){
        path_2d.waypoints.push_back(vectorXdToVector2d(x));
    }

    return path_2d;
}

amp::Path2D RRTAlgo2D::planWithFigure(const amp::Problem2D& problem){
    amp::Path path_xd = planxd(problem);

    //Recreate node map to have 2d vectors
    //std::cout << "Converting General Map to 2D Map..." << std::endl;
    for (auto const& x : node_map)
    {
        //Add the node to the 2d node map
        //std::cout << "Converting " << x.second << std::endl;
        Eigen::Vector2d q_2d = vectorXdToVector2d(x.second);
        node_map2D[x.first] = q_2d;

        //Debug
        std::cout << "Original Position: (";
        for (int i = 0; i < x.second.size(); i++){
            std::cout << x.second[i] << ", ";
        }
        std::cout << ") -> New Position (" << q_2d[0] << ", " << q_2d[1] << ")" << std::endl;
    }

    //Check the graph made by the generic planner
    amp::Visualizer::makeFigure(problem, graph, node_map2D);

    //Convert the path to 2D
    amp::Path2D path_2d;
    for (auto& x : path_xd.waypoints){
        path_2d.waypoints.push_back(vectorXdToVector2d(x));
    }

    return path_2d;
}

amp::Node RRTAlgo2D::nearestNeighbor(const Eigen::VectorXd& q_rand){
    double min_dist = INT16_MAX;
    amp::Node nearest_node;

    for (auto& x : node_map){
        //std::cout << "Key: " << x.first << std::endl;
        //std::cout << "Value: " << x.second << std::endl;

        double dist = (x.second - q_rand).norm();

        if (dist < min_dist){
            min_dist = dist;
            nearest_node = x.first;
        }
    }

    return nearest_node;

}