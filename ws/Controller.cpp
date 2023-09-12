#include "Controller.h"

// This controller is intended to be used for any bug algorithm
// It will take in a direction vector, and step the bug in that direction, checking for collisions, and adjusting 
// based off a specified distance away from the obstacle boundry.

/// @brief Takes a step in the direction specified while maintaining a boundry from the obstacles
/// @return q_next in the form of a 2D vector
Eigen::Vector2d Controller::step(const amp::Problem2D& problem, Eigen::Vector2d q_last, std::vector<Eigen::Vector2d> q_targetList, int targDex, double stepSize, double radius) const{

    //%%%%%%%%%%%%%%% Hyper Parameters %%%%%%%%%%%%%%%%%%%%
    double boundDist = 0.1; //Distance away from the obstacle boundry

    Eigen::Vector2d q_next;
    Eigen::Vector2d q_target = q_targetList[targDex];

    // Take a step in the direction of q_target
    q_next = q_last + directionVec(q_last, q_target)*stepSize;

    // Determine if a collision occured
    int collisionObjNum = collisionDetected(problem.obstacles, q_next);

    // If a collision occured, adjust the position of q_next to be the closest point on the obstacle boundry plus a predefined normal out distance
    if (collisionObjNum > -1){
        //q_next = radiusAdjust(q_last, q_next, q_targetList, targDex, stepSize, radius);
    }

    std::cout << "q_next: <" << q_next[0] << ", " << q_next[1] << "> " << std::endl;


    return q_next;
}