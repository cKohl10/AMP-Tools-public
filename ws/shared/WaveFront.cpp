#include "WaveFront.h"


//MyGridCSpace()
//    : amp::GridCSpace2D(1, 1, 0.0, 1.0, 0.0, 1.0) {}
MyGridCSpace::MyGridCSpace(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
    : amp::GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max) {}

//Finds the cell that the point given is in
std::pair<std::size_t, std::size_t> MyGridCSpace::getCellFromPoint(double x0, double x1) const {

    //Calculate the step size for each dimension
    double x0_step = (m_x0_bounds.second - m_x0_bounds.first)/size().first;
    double x1_step = (m_x1_bounds.second - m_x1_bounds.first)/size().second;

    //Calculate the cell indices and floor them
    std::size_t i = (x0 - m_x0_bounds.first)/x0_step;
    std::size_t j = (x1 - m_x1_bounds.first)/x1_step;

    //Debuging:
    //std::cout << std::endl << "New getCellFromPoint() function called!" << std::endl;
    //std::cout << "x0: " << x0 << " x1: " << x1 << std::endl;
    //std::cout << "x0_step: " << x0_step << " x1_step: " << x1_step << std::endl;
    //std::cout << "m_x0_bounds.first: " << m_x0_bounds.first << " m_x0_bounds.second: " << m_x0_bounds.second << std::endl;
    //std::cout << "m_x1_bounds.first: " << m_x1_bounds.first << " m_x1_bounds.second: " << m_x1_bounds.second << std::endl;
    //std::cout << "i: " << i << " j: " << j << std::endl << std::endl;

    return std::make_pair(i, j);
}


std::unique_ptr<amp::GridCSpace2D> MyCSpaceCtor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env){

    return std::make_unique<MyGridCSpace>(1, 1, 0.0, 1.0, 0.0, 1.0);
}


// Default ctor
MyPointWFAlgo::MyPointWFAlgo() {}

std::unique_ptr<amp::GridCSpace2D> MyPointWFAlgo::constructDiscretizedWorkspace(const amp::Environment2D& environment) {
    
    //Create the discretized parameters for translating robot
    double stepSize = 0.25;
    double cornerSize = stepSize/2;
    double x0_min = environment.x_min; 
    double x0_max = environment.x_max; 
    double x1_min = environment.y_min; 
    double x1_max = environment.y_max; 
    std::size_t x0_cells = ceil((x0_max - x0_min)/stepSize); 
    std::size_t x1_cells = ceil((x1_max - x1_min)/stepSize); 

    //Debuging:
    //std::cout << "x0_cells: " << x0_cells << " x1_cells: " << x1_cells << std::endl;
    //std::cout << "x0_min: " << x0_min << " x0_max: " << x0_max << std::endl;
    //std::cout << "x1_min: " << x1_min << " x1_max: " << x1_max << std::endl << std::endl;

    std::unique_ptr<amp::GridCSpace2D> Gptr = std::make_unique<MyGridCSpace>(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max);

    //Check that Gptr has the same values
    //std::cout << "Gptr->size().first: " << Gptr->size().first << " Gptr->size().second: " << Gptr->size().second << std::endl;

    //Check for collisions
    for (int i = 0; i < int(x0_cells); i++){
        for (int j = 0; j < int(x1_cells); j++){

            //Calculate the configuration
            double x0 = x0_min + (i)*stepSize;
            double x1 = x1_min + (j)*stepSize;
            Eigen::Vector2d pos = Eigen::Vector2d(x0, x1);
            
            //Output pos values
            //std::cout << "pos.first: " << pos.first << " pos.second: " << pos.second << std::endl;
            


            if (collisionDetectedPoint(environment.obstacles, pos) || collisionDetectedPoint(environment.obstacles, pos+Eigen::Vector2d(-cornerSize,-cornerSize)) || collisionDetectedPoint(environment.obstacles, pos+Eigen::Vector2d(-cornerSize,cornerSize)) || collisionDetectedPoint(environment.obstacles, pos+Eigen::Vector2d(cornerSize,-cornerSize)) || collisionDetectedPoint(environment.obstacles, pos+Eigen::Vector2d(cornerSize,cornerSize)) ){
                Gptr->operator()(i,j) = true;
            }
        }
    }

    return Gptr;
}

// This is just to get grade to work, you DO NOT need to override this method
// virtual amp::Path2D plan(const amp::Problem2D& problem) override {
//     std::cout << "Point Wave Front being graded with overided" << std::endl;
//     return amp::Path2D();
// }

// You need to implement here
amp::Path2D MyPointWFAlgo::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) {

    //############## Build the Grid Space as a Hash Table ###############
    HashTable2D hash;

    //Make the array of neighbors to visit
    std::vector<std::pair<int, int>> allNeighborOrder;
    bool edgeNeighbor = true;

    //Initialize the edge neighbor order
    if(edgeNeighbor){
        allNeighborOrder.push_back(std::make_pair(0, 1));
        allNeighborOrder.push_back(std::make_pair(1, 0));
        allNeighborOrder.push_back(std::make_pair(0, -1));
        allNeighborOrder.push_back(std::make_pair(-1, 0));
    } else{
        //Initialize the neighbor order
        allNeighborOrder.push_back(std::make_pair(0, 1));
        allNeighborOrder.push_back(std::make_pair(1, 1));
        allNeighborOrder.push_back(std::make_pair(1, 0));
        allNeighborOrder.push_back(std::make_pair(1, -1));
        allNeighborOrder.push_back(std::make_pair(0, -1));
        allNeighborOrder.push_back(std::make_pair(-1, -1));
        allNeighborOrder.push_back(std::make_pair(-1, 0));
        allNeighborOrder.push_back(std::make_pair(-1, 1));
    }

    //Propogate the hash table
    hash.propogateHash(q_goal, grid_cspace, allNeighborOrder);

    //Print the hashtable
    //hash.printHashTable();

    //################# Build the path #################
    amp::Path2D path;

    std::pair<int, int> startIndex = grid_cspace.getCellFromPoint(q_init[0], q_init[1]);

    //Start the path descent from the start node
    path.waypoints.push_back(q_init);

    HashNode q(startIndex, hash.getHeuristic(startIndex));

    while (q.heuristic > 2){
        //Get the next node
        q = hash.traverseHash(q, allNeighborOrder);

        //Add the node to the path
        path.waypoints.push_back(hash.getPosFromKey(q.key, grid_cspace));
    }

    return path;
}

// Default ctor
MyManipWFAlgo::MyManipWFAlgo()
    : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<CSpaceConstructor>()) {}

// You can have custom ctor params for all of these classes
MyManipWFAlgo::MyManipWFAlgo(const std::string& beep) 
    : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<CSpaceConstructor>()) {LOG("construcing... " << beep);}

// This is just to get grade to work, you DO NOT need to override this method
// virtual amp::ManipulatorTrajectory2Link plan(const LinkManipulator2D& link_manipulator_agent, const amp::Problem2D& problem) override {
//     return amp::ManipulatorTrajectory2Link();
// }

// You need to implement here
amp::Path2D MyManipWFAlgo::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace){

    //############## Build the Grid Space as a Hash Table ###############
    HashTable2D hash;

    //Make the array of neighbors to visit
    std::vector<std::pair<int, int>> allNeighborOrder;
    bool edgeNeighbor = true;

    //Initialize the edge neighbor order
    if(edgeNeighbor){
        allNeighborOrder.push_back(std::make_pair(0, 1));
        allNeighborOrder.push_back(std::make_pair(1, 0));
        allNeighborOrder.push_back(std::make_pair(0, -1));
        allNeighborOrder.push_back(std::make_pair(-1, 0));
    } else{
        //Initialize the neighbor order
        allNeighborOrder.push_back(std::make_pair(0, 1));
        allNeighborOrder.push_back(std::make_pair(1, 1));
        allNeighborOrder.push_back(std::make_pair(1, 0));
        allNeighborOrder.push_back(std::make_pair(1, -1));
        allNeighborOrder.push_back(std::make_pair(0, -1));
        allNeighborOrder.push_back(std::make_pair(-1, -1));
        allNeighborOrder.push_back(std::make_pair(-1, 0));
        allNeighborOrder.push_back(std::make_pair(-1, 1));
    }

    //Propogate the hash table
    hash.propogateHashTorus(q_goal, grid_cspace, allNeighborOrder);

    //Print the hashtable
    //hash.printHashTable();

    //################# Build the path #################
    amp::Path2D path;

    //std::vector<double> startingState = 
    //std::pair<double, double> startIndex = std::make_pair(q_init[0], q_init[1]);
    std::pair<int, int> startIndex = grid_cspace.getCellFromPoint(q_init[0], q_init[1]);
    std::pair<int, int> goalIndex = grid_cspace.getCellFromPoint(q_goal[0], q_goal[1]);

    //print the initial position and startIndex
    std::cout << "Initial position: (" << q_init[0] << ", " << q_init[1] << ")" << std::endl;
    std::cout << "Start index: (" << startIndex.first << ", " << startIndex.second << ")" << std::endl;

    //print out the goal position and index
    std::cout << "Goal position: (" << q_goal[0] << ", " << q_goal[1] << ")" << std::endl;
    std::cout << "Goal index: (" << goalIndex.first << ", " << goalIndex.second << ")" << std::endl;
    //Start the path descent from the start node
    path.waypoints.push_back(q_init);

    HashNode q(startIndex, hash.getHeuristic(startIndex));

    while (q.heuristic > 2){
        //Get the next node
        q = hash.traverseHashTorus(q, allNeighborOrder, grid_cspace);

        //print current angles
        //std::cout << "Angles-> (" << q.key.first << ", " << q.key.second << ")" << std::endl << std::endl;

        //Add the node to the path
        path.waypoints.push_back(hash.getPosFromKey(q.key, grid_cspace));
    }

    path.waypoints.push_back(q_goal);

    unwrapPath(path, Eigen::Vector2d(-M_PI, -M_PI), Eigen::Vector2d(M_PI, M_PI));

    std::cout << "Final Position Key: " << q.key.first << ", " << q.key.second << std::endl;

    return path;
}

