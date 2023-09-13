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
    amp::Problem2D problemEX = expandPolygons(problem, 0.01);
    PolyGraph polyGraph(problemEX);

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
   double goalReachedError = 0.5;
   double stepSize = 0.1;


   //%%%%%%%%%%%%%%% Variables %%%%%%%%%%%%%%%%%%%%%%%%%%%
    std::vector<Eigen::Vector2d> qL; //Vector of all leave points
    std::vector<Eigen::Vector2d> qH; //Vector of all hit points
    Eigen::Vector2d q_last; //Last movement point
    Eigen::Vector2d q_next; //Next step in path
    Eigen::Vector2d q_target; //Direction of bug when no going to goal
    Eigen::Vector2d q_goal =  problemEX.q_goal; //Goal coordinates
    amp::Obstacle2D obstacle; //Obstacle object

    int state = 1; //Determines whate state of the bug algorithm is currently active
    int collisionObjNum; //polygon number of collision, -1 if no collision
    int i = 0; //Obstacle number
    int DLdex = 1; //Index of the direction list
    bool qHitGate = false; //Determines if the bug has gone outside of the initial goalReached return zone and will allow for the bug to see qH again


   //%%%%%%%%%%%%%%% Algorithm Begins Here %%%%%%%%%%%%%%%
   std::cout << std::endl << "Starting Bug1 Algorithm..." << std::endl;
   // Let q^{L_0} = q_{start}; i = 1
    //qL.push_back(problemEX.q_init);
    path.waypoints.push_back(problemEX.q_init);

    /////////TESTING////////////
    //path.waypoints.push_back(Eigen::Vector2d(1.5, 2.0));
    //collisionDetected(problemEX.obstacles, Eigen::Vector2d(1.5, 2.0));
    /////////////////////////////

   // *Repeat:
   
    while (true){
    
        //   *Repeat until goal is reached or obstacle encountered at q^{H_i}:
        //       *from q^{L_{i-1}} move toward q_{goal}  
        while(state == 1){
            q_last = path.waypoints.back();
            q_next = q_last + directionVec(q_last, q_goal)*stepSize;
            

            // *If goal is reached:
            //    *exit
            if (goalReached(q_next, q_goal, goalReachedError)){
                path.waypoints.push_back(q_next);
                path.waypoints.push_back(q_goal);
                std::cout << "Goal Reached!" << std::endl;
                return path;
            }

            // *If obstacle encountered
            collisionObjNum = collisionDetected(problemEX.obstacles, q_next);
            if (collisionObjNum > -1){
                state = 2;
            }else{
                path.waypoints.push_back(q_next);
            }
            //std::cout << (q_goal - q_next).squaredNorm() << std::endl;
            
        }
        // After the bug has found an obstacle, it will align itself with the boundry of the obstacle, then move towards the next closest point in the vertice list
        obstacle = problemEX.obstacles[collisionObjNum];
        //Set a vector containing all the points on the obstacle path
        std::vector<Eigen::Vector2d> obstaclePath;
        std::vector<Eigen::Vector2d> directionList; //Directions for the bug to travel once it is bounding an obstacle

        //WIP: Need to define the object hit as a list of all the vertices that are not overlapping and not colliding

        directionList = closestPointOnObstacle(q_last, q_next, obstacle);
   
        //Set appropriate positions ;
        q_next = directionList[0];
        q_target = directionList[1];
        path.waypoints.push_back(q_next);
        qH.push_back(q_next);
        std::cout << "qH[" << i << "] = (" << qH[i][0] << ", " << qH[i][1] << ")" << std::endl;
        obstaclePath.push_back(q_next);

        //Reset the direction list index
        DLdex = 0;

        //Set the gate to make the bug not instantly terminate
        qHitGate = true;

        //   *Repeat until q_{goal} is reached or q^{H_i} is re-encountered
        //       *follow bountary recordeding point q^{L_i} with shortest distance to goal
        while(state == 2){
            q_last = path.waypoints.back();
            q_next = q_last + directionVec(q_last, q_target)*stepSize;

            // Goal reached
            if (goalReached(q_next, q_goal, goalReachedError)){
                path.waypoints.push_back(q_next);
                obstaclePath.push_back(q_next);
                path.waypoints.push_back(q_goal);
                obstaclePath.push_back(q_goal);
                std::cout << "Goal Reached!" << std::endl;
                return path;
            }

            // q_hit reached
            if (goalReached(q_next, qH[i], goalReachedError) && qHitGate == false){
                path.waypoints.push_back(q_next);
                obstaclePath.push_back(q_next);
                q_next = qH[i];
                path.waypoints.push_back(q_next);
                obstaclePath.push_back(q_next);
                state = 3;

                std::cout << "q_hit reached!" << std::endl;
                double dist = pathDistance(path);

                //return path if qH is the same as the last qL
                if (qL.size() > 0){
                    if (qH[i] == qL.back()){
                        return path;
                    }
                }
                continue;

                //testing
                //return path;
            }

            // next vertex target reached
            if (goalReached(q_next, q_target, goalReachedError)){
                path.waypoints.push_back(q_next);
                obstaclePath.push_back(q_next);
                q_next = q_target;

                DLdex += 1;
                q_target = directionList[DLdex%directionList.size()];

                std::cout << "Next vertex reached! Targeting (" << q_target[0] << ", " << q_target[1] << ")" << std::endl;  
                
                if (DLdex == directionList.size()-1){
                    qHitGate = false;
                }
            }

            path.waypoints.push_back(q_next);
            obstaclePath.push_back(q_next);
        }

        std::vector<Eigen::Vector2d> shortestObstaclePath = findShortestPath(obstaclePath, q_goal);
        double dist = vertVecDistance(shortestObstaclePath);

        //Define and reset state 3 variables
        qL.push_back(shortestObstaclePath.back());
        // print the coordinates of the last point in the shortest path to make sure it is not the goal
        std::cout << "qL[" << i << "] = (" << qL[i][0] << ", " << qL[i][1] << ")" << std::endl;

        q_next = shortestObstaclePath[0];
        q_target = shortestObstaclePath[1];
        DLdex = 1;

        //   *If goal is reached
        //       *exit
        while(state == 3){
            //   *Go to q^{L_i}
            q_last = path.waypoints.back();
            //std::cout << "q_last = (" << q_last[0] << ", " << q_last[1] << ")" << std::endl;

            q_next = shortestObstaclePath[DLdex];


            //   *If move toward q_{goal} moves into obstacle
            //       *exit with failure
            if (goalReached(q_next, q_goal, goalReachedError)){
                path.waypoints.push_back(q_next);
                path.waypoints.push_back(q_goal);
                std::cout << "Goal Reached!" << std::endl;
                return path;
            }

            // q_L reached
            if (goalReached(q_next, qL[i], goalReachedError)){
                path.waypoints.push_back(q_next);
                q_next = qL[i];
                path.waypoints.push_back(q_next);
                state = 1;

                std::cout << "q_leave reached!" << std::endl;
                double dist = pathDistance(path);
                i++;
                continue;

                //testing
                //return path;
            }

            /*            // next vertex target reached
            if (goalReached(q_next, q_target, goalReachedError) && DLdex < shortestObstaclePath.size()-1){
                path.waypoints.push_back(q_next);
                q_next = q_target;

                
                q_target = shortestObstaclePath[DLdex];

                std::cout << "Next vertex reached! Targeting (" << q_target[0] << ", " << q_target[1] << ")" << std::endl;  
            } else{
                std::cout << "Error: Incorrectly terminated state 3" << std::endl;
                return path;
            }*/

            //   *Else
            //       *i=i+1
            //       *continue  

            DLdex += 1;
            path.waypoints.push_back(q_next);
            
        }

    }
    
    return path;
}



