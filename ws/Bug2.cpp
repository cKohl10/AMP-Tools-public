#include "Bug1.h"

//Eigen::Vector2d directionVec(Eigen::Vector2d fromPoint, Eigen::Vector2d toPoint);
//bool goalReached(Eigen::Vector2d currentPoint, Eigen::Vector2d q_goal, double goalReachedError);

// Implement your methods in the `.cpp` file, for example:
amp::Path2D Bug1::plan(const amp::Problem2D& problem) const{

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;

    // Handle overlapping polygons
    //print status statement
    
    std::cout << std::endl << "Expanding polygons..." << std::endl;
    amp::Problem2D problemEX = expandPolygons(problem, 0.01); //Double defines expansion size
    PolyGraph polyGraph(problemEX);

    /*
    std::cout << std::endl << "Sorting polygons into graph..." << std::endl;
    polyGraph.sort();

    std::cout << std::endl << "Seperating by overlapping vertices..." << std::endl;
    polyGraph.findVertices();

    std::cout << std::endl << "Merging overlapping polygons..." << std::endl;
    polyGraph.commonContour();

    std::cout << std::endl << "Creating new workspace" << std::endl;
    //Clear all of problemEX's obstacles
    problemEX.obstacles.clear();

    //Add the new obstacles to problemEX
    for (int i = 0; i < polyGraph.vertices.size(); i++){

        //print obstacle number
        std::cout << "Obstacle " << i << " vertices: " << std::endl;


        amp::Obstacle2D newObstacle(polyGraph.newObstacleVerticesSet[i]);
        problemEX.obstacles.push_back(newObstacle);

        //print vertices
        std::vector<Eigen::Vector2d> vertices = problemEX.obstacles[i].verticesCCW();
        for (int j = 0; j < vertices.size(); j++){
            std::cout << "(" <<  vertices[j][0] << ", " << vertices[j][1] << ")" << std::endl;
        }
    }
    */


    //path.waypoints.push_back(problemEX.q_init);
    //path.waypoints.push_back(Eigen::Vector2d(1.0, 10.0));
    //path.waypoints.push_back(Eigen::Vector2d(3.0, 9.0));
    //path.waypoints.push_back(problemEX.q_goal);

    // Print all obstacles and their verticies
    /*
    for (amp::Obstacle2D o : problemEX.obstacles){
        o.print();
    }
    */

   //%%%%%%%%%%%%%%% Hyper Parameters %%%%%%%%%%%%%%%%%%%%
   double initialDist = (problem.q_goal - problem.q_init).norm();
    double stepSize = initialDist/1000;
    double goalReachedError = stepSize/2;
    double maxDist = 1000;


   //%%%%%%%%%%%%%%% Variables %%%%%%%%%%%%%%%%%%%%%%%%%%%
    std::vector<Eigen::Vector2d> qL; //Vector of all leave points
    std::vector<Eigen::Vector2d> qH; //Vector of all hit points
    Eigen::Vector2d q_last; //Last movement point
    Eigen::Vector2d q_next; //Next step in path
    Eigen::Vector2d q_target; //Direction of bug when no going to goal
    Eigen::Vector2d q_goal = problemEX.q_goal; //Goal coordinates
    amp::Obstacle2D obstacle; //Obstacle object

    //Defining the M-Line
    Eigen::ParametrizedLine<double, 2> MLine(problemEX.q_init, (q_goal - problemEX.q_init).normalized());

    Controller controller(problemEX);

    int state = 1; //Determines whate state of the bug algorithm is currently active
    int collisionObjNum; //polygon number of collision, -1 if no collision
    int i = 0; //Obstacle number
    int DLdex = 1; //Index of the direction list
    //bool qHitGate = false; //Determines if the bug has gone outside of the initial goalReached return zone and will allow for the bug to see qH again


   //%%%%%%%%%%%%%%% Algorithm Begins Here %%%%%%%%%%%%%%%
   std::cout << std::endl << "Starting Bug 2 Algorithm..." << std::endl;
   // Let q^{L_0} = q_{start}; i = 1
    //qL.push_back(problemEX.q_init);
    path.waypoints.push_back(problemEX.q_init);

    while (true){

        //State 1: Move to goal on M-Line
        while(state == 1){
            q_last = path.waypoints.back();
            q_next = controller.step(q_last, stepSize);
            

            // *If goal is reached:
            //    *exit
            if (goalReached(q_next, q_goal, goalReachedError)){
                path.waypoints.push_back(q_next);
                path.waypoints.push_back(q_goal);
                std::cout << "Goal Reached!" << std::endl;
                //print out the distance traveled
                std::cout << "Distance traveled: " << pathDistance(path) << std::endl;
                return path;
            }

            // *If obstacle encountered
            if (controller.getCurrentFollowObstacle() > -1){
                state = 2;
            }else{
                // *If no obstacle encountered, add to path
                path.waypoints.push_back(q_next);
            }

            // *If max distance traveled,  exit
            if (controller.distTraveled > maxDist){
                return path;
            }
            
        }

    }
    return path;
}



