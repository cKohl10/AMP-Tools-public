#include "Controller.h"

Controller::Controller(amp::Problem2D p, bool rightTurning){
    problem = p;
    distTraveled = 0;
    targetObstacleIndex = -1;
    rightSideOfMLine = false;
    targetQueue.push_back(problem.q_goal);
    leftTurningBug = rightTurning;
}

//Prints out all the vertices in the target queue starting from the first element
void Controller::printtargetQueue(){
    //send the new target vertices to the target queue
    std::cout << std::endl << "####### First in Target Queue #######" << std::endl;
    for (int i = targetQueue.size()-1; i >= 0; i--){
        //print out new target list
        std::cout << "Index: " << i << ", (" << targetQueue[i].x() << ", " << targetQueue[i].y() << ")" << std::endl;
    }
    std::cout << "####### Last in Target Queue #######" << std::endl << std::endl;
}

// For manually changing the follow target obstacle index
// Does not change target queue
void Controller::setFollowTarget(int obstacleIndex){
    //print out the new follow target
    if (obstacleIndex == -1){
        std::cout << "New follow target: Goal" << std::endl;
        obstaclePathTaken.clear();
        clearTargetQueue();
    } else{
        std::cout << "New follow target: " << std::endl;
        std::cout << "ERROR BROKEN DO NOT USE" << std::endl;
    }


    targetObstacleIndex = obstacleIndex;
    return;
}

// Removes all targets and sets them to the goal
void Controller::clearTargetQueue(){

    //print out the new follow target
    std::cout << "New follow target: Goal" << std::endl;

    targetQueue.clear();
    targetQueue.push_back(problem.q_goal);
    return;
}

Eigen::Vector2d Controller::getCurrentTarget(){
    return targetQueue.back();
}

//Makes the bug take a step in the direction of the next target. If it collides with an obstacle that is not the target it is following,
//it will push the vertices of that obstacle onto the 
Eigen::Vector2d Controller::step(Eigen::Vector2d q_last, double stepSize){
    Eigen::Vector2d q_target = targetQueue.back();
    Eigen::Vector2d q_next = q_last + directionVec(q_last, q_target)*stepSize;

    //Print out all member variables
    /*
    std::cout << "q_last: (" << q_last.x() << ", " << q_last.y() << ")" << std::endl;
    std::cout << "q_target: (" << q_target.x() << ", " << q_target.y() << ")" << std::endl;
    std::cout << "q_next: (" << q_next.x() << ", " << q_next.y() << ")" << std::endl;
    std::cout << "targetObstacleIndex: " << targetObstacleIndex << std::endl << std::endl;
    */


    //Check for collision with an obstacle
    std::vector<int> collisionDexs = collisionDetected(problem.obstacles, q_next);
    //std::cout << "Made it here" << std::endl;

    //If the bug is following an obstacle, check if it collides with a different obstacle than it is following
    for (int j = 0; j < collisionDexs.size(); j++){
        if ((collisionDexs[j] != targetObstacleIndex) && (collisionDexs[j] > -1)){
            
            //print out what objects have been collided with
            std::cout << "Collision with obstacle " << collisionDexs[j] << std::endl;

            //Set new target obstacle index
            targetObstacleIndex = collisionDexs[j];

            //Clear current target queue
            targetQueue.clear();

            //Set target queue to the vertices of the new target obstacle
            std::vector<Eigen::Vector2d> newTargList = closestPointOnObstacle(q_last, q_next, problem.obstacles[collisionDexs[j]]);
            //Check to see which direction bug should move in:
            if (leftTurningBug == false){
                //reverse the target list
                std::cout << "Right turning robot... reversing target list" << std::endl;
                std::reverse(newTargList.begin(), newTargList.end());
            }
            //print contents of new target list
            /*
            std::cout << std::endl << "New target list: " << std::endl;
            for (int i = 0; i < newTargList.size(); i++){
                //print out new target list
                std::cout << "(" << newTargList[i].x() << ", " << newTargList[i].y() << ")" << std::endl;
            }
            */

            if (newTargList.size() > 0){
                //print out newTargetList size
                //std::cout << std::endl << "New target list size: " << newTargList.size() << std::endl;

                
                //send the new target vertices to the target queue
                for (int i = 0; i < newTargList.size(); i++){

                    
                    if (i == newTargList.size()-1 && leftTurningBug){
                        // q_next is the intersection point of the line from q_last to q_target and the obstacle
                        q_next = newTargList[i];
                    }else if((i == 0 && leftTurningBug==false)){
                        // q_next is the intersection point of the line from q_last to q_target and the obstacle.
                        // Since the target list was reversed, it is now the first index of the new target list
                        q_next = newTargList[i];
            
                    } else{
                        targetQueue.push_back(newTargList[i]);
                    }

                }

                //printtargetQueue();
                
            } else {
                std::cout << "ERROR: No new target list" << std::endl;
            }

            break;

        }
    }

    //Adjust the target queue after checking for collisions
    //q_target = targetQueue.front();
    //q_next = q_last + directionVec(q_last, q_target)*stepSize;

    if (targetObstacleIndex > -1){
        obstaclePathTaken.push_back(q_next);
    }

    distTraveled += (q_next-q_last).norm();
    //print distance traveled
    //std::cout << distTraveled << std::endl;

    return q_next;
}

//Adds a point to the obstacle path taken
void Controller::logPointInObstaclePath(Eigen::Vector2d q){
    obstaclePathTaken.push_back(q);
    return;
}

std::vector<Eigen::Vector2d> Controller::getObstaclePathTaken(){
    return obstaclePathTaken;
}

int Controller::getCurrentFollowObstacle(){
    return targetObstacleIndex;
}

std::vector<Eigen::Vector2d> Controller::getTargetQueue(){
    return targetQueue;
}

void Controller::nextTarget(){
    if (targetQueue.size() > 1){
        targetQueue.pop_back();
    } else{
        //print error message
        std::cout << "ERROR: Target queue is empty" << std::endl;
    }
    return;
}

void Controller::clearBug(){
    targetObstacleIndex = -1;
    targetQueue.clear();
    targetQueue.push_back(problem.q_goal);
    obstaclePathTaken.clear();
    return;
}

//Checks if the M-Line was crossed, if so, return true
bool Controller::checkMLineIntersection(Eigen::ParametrizedLine<double, 2> mLine, Eigen::Vector2d q_goal, Eigen::Vector2d q){

    // Compare location with the normal of the M-Line
    Eigen::Vector2d normal(mLine.direction()[1], -mLine.direction()[0]);
    bool onRightSide = (normal.dot(q - q_goal) > 0);

    //Debugging which side the bug is on
    /*
    if (onRightSide){
        std::cout << "On the RIGHT side of the M-Line" << std::endl;
    } else{
        std::cout << "On the LEFT side of the M-Line" << std::endl;
    }*/

    // If the bug is on the other side of the M-Line, then it has crossed the M-Line
    if (onRightSide != rightSideOfMLine){
        rightSideOfMLine = !rightSideOfMLine;
        return true;
    }

    return false;
}