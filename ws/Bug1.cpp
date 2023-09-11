#include "Bug1.h"

//Eigen::Vector2d directionVec(Eigen::Vector2d fromPoint, Eigen::Vector2d toPoint);
//bool goalReached(Eigen::Vector2d currentPoint, Eigen::Vector2d q_goal, double goalReachedError);

// Implement your methods in the `.cpp` file, for example:
amp::Path2D Bug1::plan(const amp::Problem2D& problem) const{

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;
    //path.waypoints.push_back(problem.q_init);
    //path.waypoints.push_back(Eigen::Vector2d(1.0, 10.0));
    //path.waypoints.push_back(Eigen::Vector2d(3.0, 9.0));
    //path.waypoints.push_back(problem.q_goal);

    // Print all obstacles and their verticies
    /*
    for (amp::Obstacle2D o : problem.obstacles){
        o.print();
    }
    */

   //%%%%%%%%%%%%%%% Hyper Parameters %%%%%%%%%%%%%%%%%%%%
   double goalReachedError = 0.5;
   double stepSize = 1;


   //%%%%%%%%%%%%%%% Variables %%%%%%%%%%%%%%%%%%%%%%%%%%%
    std::vector<Eigen::Vector2d> qL; //Vector of all leave points
    std::vector<Eigen::Vector2d> qH; //Vector of all hit points
    std::vector<Eigen::Vector2d> obstaclePath;
    Eigen::Vector2d q_last; //Last movement point
    Eigen::Vector2d q_next; //Next step in path
    Eigen::Vector2d q_goal =  problem.q_goal; //Goal coordinates

    int state = 1; //Determines whate state of the bug algorithm is currently active
    int collisionObjNum; //polygon number of collision, -1 if no collision
    int i = 0; //Obstacle number


   //%%%%%%%%%%%%%%% Algorithm Begins Here %%%%%%%%%%%%%%%
   // Let q^{L_0} = q_{start}; i = 1
    qL.push_back(problem.q_init);
    path.waypoints.push_back(problem.q_init);

    /////////TESTING////////////
    //path.waypoints.push_back(Eigen::Vector2d(1.5, 2.0));
    //collisionDetected(problem.obstacles, Eigen::Vector2d(1.5, 2.0));
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
                return path;
            }

            // *If obstacle encountered
            path.waypoints.push_back(q_next);
            collisionObjNum = collisionDetected(problem.obstacles, q_next);
            if (collisionObjNum > 0){
                state = 2;
            }
            //std::cout << (q_goal - q_next).squaredNorm() << std::endl;

        }

        // After the bug has found an obstacle, it will align itself with the boundry of the obstacle, then move towards the next closest point in the vertice list
        

        //   *Repeat until q_{goal} is reached or q^{H_i} is re-encountered
        //       *follow bountary recordeding point q^{L_i} with shortest distance to goal
        while(state == 2){
            q_last = path.waypoints.back();
            q_next = q_last + directionVec(q_last, q_goal)*stepSize;


        //   *If goal is reached
        //       *exit

        //   *Go to q^{L_i}

        //   *If move toward q_{goal} moves into obstacle
        //       *exit with failure

        //   *Else
        //       *i=i+1
        //       *continue  
        }

    }
    
    return path;
}



