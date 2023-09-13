#include "Controller.h"

Controller::Controller(amp::Problem2D p){
    problem = p;
    targetObstacleIndex = -1;
    targetQueue.push_back(problem.q_goal);
}

//Prints out all the vertices in the target queue starting from the first element
void Controller::printtargetQueue(){
    std::cout << "Target Queue: " << std::endl;
    for (int i = 0; i < targetQueue.size(); i++){
        std::cout << targetQueue[i] << std::endl;
    }
}

// For manually changing the follow target obstacle index
// Does not change target queue
void Controller::setFollowTarget(int obstacleIndex){
    //print out the new follow target
    if (obstacleIndex == -1){
        std::cout << "New follow target: Goal" << std::endl;
    } else{
    std::cout << "New follow target: " << std::endl;
    }


    targetObstacleIndex = obstacleIndex;
    return;
}

// Removes all targets and sets them to the goal
void Controller::clearTargetQueue(){
    targetQueue.clear();
    targetQueue.push_back(problem.q_goal);
    return;
}

Eigen::Vector2d Controller::getCurrentTarget(){
    return targetQueue.front();
}

//Makes the bug take a step in the direction of the next target. If it collides with an obstacle that is not the target it is following,
//it will push the vertices of that obstacle onto the 
Eigen::Vector2d Controller::step(Eigen::Vector2d q_last, double stepSize){
    Eigen::Vector2d q_target = targetQueue.front();
    Eigen::Vector2d q_next = q_last + directionVec(q_last, q_target)*stepSize;

    return q_next;
}