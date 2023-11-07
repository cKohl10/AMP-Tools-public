#include "RRT.h"

RRTAlgo2D::RRTAlgo2D(double r, double p_goal, int n, double epsilon, Eigen::Vector2d xbounds, Eigen::Vector2d ybounds){
    this->r = r;
    this->p_goal = p_goal;
    this->n = n;
    this->epsilon = epsilon;
    this->bounds = {xbounds, ybounds};
}

RRTAlgo2D::RRTAlgo2D(){
    //Default values
    this->r = 0.5;
    this->p_goal = 0.05;
    this->n = 5000;
    this->epsilon = 0.25;
    this->bounds = {Eigen::Vector2d(-10.0, 10.0), Eigen::Vector2d(-10.0, 10.0)};
}

amp::Path RRTAlgo2D::planxd(const amp::Problem2D& problem){

    //Debug
    //std::cout << "Running RRT Algorithm..." << std::endl;

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
            //std::cout << "Sampling Random Node..." << std::endl;
            //Sample random node location
            for (int j = 0; j < bounds.size(); j++) {   
                q_rand[j] = randomDouble(bounds[j][0], bounds[j][1]);
            }
        } else{
            //std::cout << "Sampling Goal Node..." << std::endl;
            //Sample goal node location
            q_rand = q_goal;
        }

        //Find the nearest node to the random node
        //std::cout << "Finding Nearest Node..." << std::endl;
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
                std::reverse(path_nd.waypoints.begin(), path_nd.waypoints.end());

                //Return the path
                return path_nd;
            }

            node_counter++;

        }
        //std::cout << "Node " << node_counter << "/" << n << " Created" << std::endl;

    }
    
    return path_nd;
}

amp::Path2D RRTAlgo2D::plan(const amp::Problem2D& problem){
    //Update Bounds
    Eigen::Vector2d xbounds = {problem.x_min, problem.x_max};
    Eigen::Vector2d ybounds = {problem.y_min, problem.y_max};
    bounds[0] = xbounds;
    bounds[1] = ybounds;

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
        // std::cout << "Original Position: (";
        // for (int i = 0; i < x.second.size(); i++){
        //     std::cout << x.second[i] << ", ";
        // }
        // std::cout << ") -> New Position (" << q_2d[0] << ", " << q_2d[1] << ")" << std::endl;
    }

    //Convert the path to 2D
    amp::Path2D path_2d;
    for (auto& x : path_xd.waypoints){
        path_2d.waypoints.push_back(vectorXdToVector2d(x));
    }

    amp::Visualizer::makeFigure(problem, path_2d);

    //Check the graph made by the generic planner
    amp::Visualizer::makeFigure(problem, graph, node_map2D);

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

//################ Multi Agent Centralized RRT ################
MACentralized::MACentralized(){
    this->r = 0.5;
    this->p_goal = 0.05;
    this->n = 20000;
    this->epsilon = 0.5;
    this->bounds = {Eigen::Vector2d(-10.0, 10.0), Eigen::Vector2d(-10.0, 10.0)};
}

MACentralized::MACentralized(double r, double p_goal, int n, double epsilon, int m){
    this->r = r;
    this->p_goal = p_goal;
    this->n = n;
    this->epsilon = epsilon;
    this->m = m;
    this->bounds = {Eigen::Vector2d(-10.0, 10.0), Eigen::Vector2d(-10.0, 10.0)};
}


amp::MultiAgentPath2D MACentralized::plan(const amp::MultiAgentProblem2D& problem){

    //Update based on the problem
    Eigen::Vector2d xbounds = {problem.x_min, problem.x_max};
    Eigen::Vector2d ybounds = {problem.y_min, problem.y_max};
    bounds[0] = xbounds;
    bounds[1] = ybounds;
    radius = problem.agent_properties[0].radius;
    m = problem.agent_properties.size();

    //std::cout << std::endl << "Running Multi Agent Centralized RRT Algorithm with " << m << " robots"  << std::endl;

    //Adjust for if the edge is farther out than the bounds

    //resize obstacles
    //std::vector<amp::Obstacle2D> obstacles = expandPolygons(problem.obstacles, 0.5);
    //std::vector<amp::Obstacle2D> obstacles = expandBoxes(problem.obstacles, 0.5);
    std::vector<amp::Obstacle2D> obstacles = expandPolygonsByEdge(problem.obstacles, 0.5);

    //Get the working set of robots
    std::vector<amp::CircularAgentProperties> agent_properties;
    for (int i = 0; i < m; i++){
        agent_properties.push_back(problem.agent_properties[i]);
    }

    //Debug

    //Make Path Object
    amp::MultiAgentPath2D ma_path;

    //Initialize the root node
    amp::Node root_node = 0;
    std::vector<Eigen::Vector2d> q_root;

    //Check if starting incollision
    if(!isSubpathCollisionFree(q_root, q_root, obstacles)){
        std::cout << "Starting in Collision!" << std::endl;
        return ma_path;
    }

    //Assign all the initial states to the root node
    for (auto& x : agent_properties) q_root.push_back(x.q_init);
    ma2d_node_map[root_node] = q_root;

    //Define a size for the future nodes
    int node_size = q_root.size();
    //printState(q_root, "Root Node");

    //Initialize the goal node
    amp::Node goal_node = 1;
    std::vector<Eigen::Vector2d> q_goal;
    for (auto& x :agent_properties) q_goal.push_back(x.q_goal);

    //Inialize node counter 2 is the right number
    //amp::Node node_counter = 100000;
    amp::Node node_counter = 2;

    //Initialize loop counter
    int loop_counter = 0;

    //Start search loop
    while (node_counter < n && loop_counter < n*10){

        //Create a new node the same dimension as the init_state
        std::vector<Eigen::Vector2d> q_rand;

        //Sample random node with probability p_goal
        if (randomDouble(0, 1) > p_goal){
            //std::cout << "Sampling Random Node..." << std::endl;

            //Sample random node location
            for (int j = 0; j < node_size; j++) {
                Eigen::Vector2d q_rand_2d = {randomDouble(bounds[0][0], bounds[0][1]), randomDouble(bounds[1][0], bounds[1][1])};
                q_rand.push_back(q_rand_2d);
            }

        } else{
            //std::cout << "Sampling Goal Node..." << std::endl;
            //Sample goal node location
            q_rand = q_goal;
        }

        //Find the nearest node to the random node
        //std::cout << "Finding Nearest Node..." << std::endl;
        amp::Node nearest_node = nearestNeighbor(q_rand);
        std::vector<Eigen::Vector2d> q_near = ma2d_node_map[nearest_node];

        //Create a new node in the direction of the random node
        std::vector<Eigen::Vector2d> q_new;
        for (int i = 0; i < q_near.size(); i++) q_new.push_back(q_near[i] + (q_rand[i] - q_near[i]).normalized() * r);

        //Check if the new node is in collision
        if (isSubpathCollisionFree(q_near, q_new, obstacles)){
            
            //Add the new node to the node map
            ma2d_node_map[node_counter] = q_new;

            //Add the new node to the graph
            graph.connect(nearest_node, node_counter, r);

            
            if (isSystemInGoal(q_new, q_goal)){
                //std::cout << "Goal Reached!" << std::endl;

                //Add the goal node to the node map
                ma2d_node_map[goal_node] = q_goal;

                //Add the goal node to the graph
                graph.connect(node_counter, goal_node, r);

                //Build the path
                ma_path.agent_paths = backoutPath(goal_node);
    
                return ma_path;
            }
            

            node_counter++;
            //std::cout << "Node " << node_counter << "/" << n << " Created" << std::endl;

        }

        loop_counter++;

    }

    //Check the graph made by the generic planner
    //amp::Visualizer::makeFigure(problem, graph, ma2d_node_map);

    //############## DEBUG ################
    //std::cout << "Goal not found! Backing out path..." << std::endl;
    // for (int j = 0; j < m; j++){
    //     ma_path.agent_paths.push_back(amp::Path2D());
    // }

    // //Get the closest node to the goal
    amp::Node nearest_node = nearestNeighbor(q_goal);
        std::cout << "Nearest Node: " << nearest_node << " is (";
    for (int i = 0; i < q_goal.size()-1; i++) std::cout << (ma2d_node_map[nearest_node][i] - q_goal[i]).norm() << ", ";
    std::cout << (ma2d_node_map[nearest_node][q_goal.size()-1] - q_goal[q_goal.size()-1]).norm();
    std::cout << ") away from goal state" <<std::endl;

    //Add the goal node to the node map in case the nearest was close
    ma2d_node_map[goal_node] = q_goal;

    //Add the goal node to the graph
    graph.connect(nearest_node, goal_node, r);
    ma_path.agent_paths = backoutPath(nearest_node);
    //#######################################
    
    return ma_path;
};

std::vector<Eigen::Vector2d> MACentralized::decompose2D(Eigen::VectorXd q){
    std::vector<Eigen::Vector2d> q_2d;
    int n = q.size()/2;

    for (int i = 0; i < n; i++){
        Eigen::Vector2d q_i;
        q_i << q[2*i], q[2*i+1];
        q_2d.push_back(q_i);
    }

    return q_2d;
}

Eigen::VectorXd MACentralized::compose2D(std::vector<Eigen::Vector2d> q){
    Eigen::VectorXd q_nd;
    int n = q.size();

    for (int i = 0; i < n; i++){
        q_nd.conservativeResize(2*i+2);
        q_nd[2*i] = q[i][0];
        q_nd[2*i+1] = q[i][1];
    }

    return q_nd;
}

amp::Node MACentralized::nearestNeighbor(const std::vector<Eigen::Vector2d>& q_rand){
    double min_dist = INT16_MAX;
    amp::Node nearest_node;

    for (auto& x : ma2d_node_map){
        //std::cout << "Key: " << x.first << std::endl;
        //std::cout << "Value: " << x.second << std::endl;

        double dist = 0;
        for (int i = 0; i < x.second.size(); i++) dist = dist + (x.second[i] - q_rand[i]).norm();

        if (dist < min_dist){
            min_dist = dist;
            nearest_node = x.first;
        }
    }

    return nearest_node;
}

bool MACentralized::isSubpathCollisionFree(const std::vector<Eigen::Vector2d>& q_near, const std::vector<Eigen::Vector2d>& q_new, const std::vector<amp::Obstacle2D>& obstacles){
    //return true;
    
    //Check if the paths are collision free against obstacles in the problem
    for (int i = 0; i < q_new.size(); i++){
        if (polyToPolyCollision(obstacles, q_near[i], q_new[i], radius)){
            return false;
        }
    }

    //Check if the paths are collision free against other robots
    for (int i = 0; i < q_new.size(); i++){
        for (int j = 0; j < q_new.size(); j++){
            if (i != j){
                if ((q_new[i] - q_new[j]).norm() < 2.5*radius){
                    return false;
                }
            }
        }
    }

    //If the paths are collision free, return true
    return true;
}

bool MACentralized::isSystemInGoal(std::vector<Eigen::Vector2d> q, std::vector<Eigen::Vector2d> q_goal){
    for (int i = 0; i < q.size(); i++){
        if ((q[i] - q_goal[i]).norm() > epsilon) return false;
    }
    return true;
}

std::vector<amp::Path2D> MACentralized::backoutPath(amp::Node final_node){
    std::vector<amp::Path2D> paths;

    //Build the path by repeating the path and forming a vector of path2ds   
    for(int i = 0; i < m; i++){

        //Start from the given node
        amp::Node curr_node = final_node;

        //Make new path
        amp::Path2D path_2d;

        while (curr_node != 0){
            path_2d.waypoints.push_back((ma2d_node_map[curr_node])[i]);
            curr_node = graph.parents(curr_node)[0];
        }
        path_2d.waypoints.push_back((ma2d_node_map[curr_node])[i]);
        std::reverse(path_2d.waypoints.begin(), path_2d.waypoints.end());

        //Add the path to the vector of paths
        paths.push_back(path_2d);
    }

    for (int j = m; j < m; j++){
        amp::Path2D path_2d;
        paths.push_back(path_2d);
    }
    return paths;
}

void MACentralized::printState(std::vector<Eigen::Vector2d> state, std::string name){
    std::cout << name << ": ";
    for (int i = 0; i < state.size(); i++){
        std::cout << "(" << state[i][0] << ", " << state[i][1] << ") ";
    }
    std::cout << std::endl;
}

int MACentralized::getTreeSize(){
    return ma2d_node_map.size();
}



// ###################### MULTI AGENT DECENTRALZIED #############################
MADecentralized::MADecentralized(){
    this->r = 0.5;
    this->p_goal = 0.05;
    this->n = 7500;
    this->epsilon = 0.5;
    //this->m_max = 100;
    this->bounds = {Eigen::Vector2d(-10.0, 10.0), Eigen::Vector2d(-10.0, 10.0)};
}

MADecentralized::MADecentralized(double r, double p_goal, int n, double epsilon, int m){
    this->r = r;
    this->p_goal = p_goal;
    this->n = n;
    this->epsilon = epsilon;
    this->m = m;
    this->bounds = {Eigen::Vector2d(-10.0, 10.0), Eigen::Vector2d(-10.0, 10.0)};
}

amp::MultiAgentPath2D MADecentralized::plan(const amp::MultiAgentProblem2D& problem){
        //Debug
    //std::cout << "Running RRT Algorithm..." << std::endl;

        //Update based on the problem
    Eigen::Vector2d xbounds = {problem.x_min, problem.x_max};
    Eigen::Vector2d ybounds = {problem.y_min, problem.y_max};
    bounds[0] = xbounds;
    bounds[1] = ybounds;
    radius = problem.agent_properties[0].radius;
    m = problem.agent_properties.size();

    //std::cout << std::endl << "Running Multi Agent Centralized RRT Algorithm with " << m << " robots"  << std::endl;

    //resize obstacles
    //std::vector<amp::Obstacle2D> obstacles = expandPolygons(problem.obstacles, 0.5);
    //std::vector<amp::Obstacle2D> obstacles = expandBoxes(problem.obstacles, 0.5);
    std::vector<amp::Obstacle2D> obstacles = expandPolygonsByEdge(problem.obstacles, 0.5);

    //Plan for each robot
    amp::MultiAgentPath2D ma_path;
    for (int i = 0; i < m; i++){
        //std::cout << "Planning for Robot " << i << std::endl;
        amp::Path2D path = GoalBiasedRRT(problem.agent_properties[i].q_init, problem.agent_properties[i].q_goal, obstacles);

        ma_path.agent_paths.push_back(path);
    }

    return ma_path;
}

amp::Path2D MADecentralized::GoalBiasedRRT(const Eigen::Vector2d& init_state, const Eigen::Vector2d& q_goal, std::vector<amp::Obstacle2D> obstacles){
    //Make Path Object
    amp::Path2D path;
    node_map.clear();
    time_map.clear();
    amp::Graph<double> graph;

    //Start the timing for the algorithm
    int time_counter = 0;

    //Initialize the root node
    amp::Node root_node = 0;
    node_map[root_node] = init_state;
    time_map[root_node] = time_counter;

    //Initialize the goal node
    amp::Node goal_node = 1;

    //Inialize node counter
    amp::Node node_counter = 2;

    //Initialize Loop Counter
    int loop_counter = 0;

    //Start search loop
    while (node_counter < n && loop_counter < n*10){

        //Create a new node the same dimension as the init_state
        Eigen::Vector2d q_rand;

        //Sample random node with probability p_goal
        if (randomDouble(0, 1) > p_goal){
            //std::cout << "Sampling Random Node..." << std::endl;
            //Sample random node location
            for (int j = 0; j < q_rand.size(); j++) {   
                q_rand[j] = randomDouble(bounds[j][0], bounds[j][1]);
            }
        } else{
            //std::cout << "Sampling Goal Node..." << std::endl;
            //Sample goal node location
            q_rand = q_goal;
        }

        //Find the nearest node to the random node
        //std::cout << "Finding Nearest Node..." << std::endl;
        amp::Node nearest_node = nearestNeighbor(q_rand);
        Eigen::Vector2d q_near = node_map[nearest_node];
        int time_near = time_map[nearest_node];

        //Create a new node in the direction of the random node
        Eigen::Vector2d q_new = q_near + (q_rand - q_near).normalized() * r;
        int time_new = time_near + 1;

        //Check if the new node is in collision
        // NOTE: This is a 2D collision checker
        if (isSubpathCollisionFree(q_near, q_new, obstacles, time_new)){
            
            //Add the new node to the node map
            node_map[node_counter] = q_new;
            time_map[node_counter] = time_new;

            //Add the new node to the graph
            graph.connect(nearest_node, node_counter, r);

            //Check if the new node is within the termination radius
            if ((q_new - q_goal).norm() < epsilon){

                //std::cout << "Goal Reached!" << std::endl;

                //Add the goal node to the node map
                node_map[goal_node] = q_goal;
                time_map[goal_node] = time_new+1;

                //Add the goal node to the graph
                graph.connect(node_counter, goal_node, (q_new-q_goal).norm());

                //Build the path
                amp::Node curr_node = goal_node;
                int steps = 0;
                while (curr_node != 0){
                    path.waypoints.push_back(node_map[curr_node]);
                    curr_node = graph.parents(curr_node)[0];
                }
                path.waypoints.push_back(node_map[curr_node]);
                std::reverse(path.waypoints.begin(), path.waypoints.end());

                for (int i = 0; i < path.waypoints.size()-1; i++){
                    time_of_others_map[i].push_back(path.waypoints[i]);
                }

                //Return the path
                return path;
            }

            node_counter++;

        }
        //std::cout << "Node " << node_counter << "/" << n << " Created" << std::endl;
        loop_counter++;

    }
    
    //std::cout << "Goal not found!" << std::endl;
    return path;
}

bool MADecentralized::isSubpathCollisionFree(const Eigen::Vector2d& q_near, const Eigen::Vector2d& q_new, const std::vector<amp::Obstacle2D>& obstacles, int time){

    //Check if the paths are collision free against obstacles in the problem
    if (polyToPolyCollision(obstacles, q_near, q_new, radius)){
        return false;
    }

    //Check if the paths are collision free against other robots
    std::vector<Eigen::Vector2d> q_others = time_of_others_map[time];
    for (Eigen::Vector2d q : q_others){
        if ((q_new - q).norm() < 4*radius){
            return false;
        }
    }

    //If the paths are collision free, return true
    return true;
}

amp::Node MADecentralized::nearestNeighbor(const Eigen::Vector2d& q_rand){
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