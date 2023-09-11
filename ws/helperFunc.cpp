#include "helperFunc.h"
#include "Eigen/Geometry"


/// @return the normalized distance between two 2D vector points
Eigen::Vector2d directionVec(Eigen::Vector2d fromPoint, Eigen::Vector2d toPoint){
    Eigen::Vector2d v = (toPoint - fromPoint).normalized();
    return v;
}

/// @brief Determines if the current point of the bug is within range of the goal to terminate
/// @return A bool where 1 = goal has been reached
bool goalReached(Eigen::Vector2d currentPoint, Eigen::Vector2d q_goal, double goalReachedError){
    double distance = (q_goal - currentPoint).squaredNorm();
    if (distance <= goalReachedError){
        return 1;
    }
    return 0;
}

/// @brief This builds a union of primitives for each obstacle and compares the location of the bug 
/// @param obstacles 
/// @param position 
/// @return -1 for no collision, or the object number in the list that was collided with
int collisionDetected(std::vector<amp::Obstacle2D> obstacles, Eigen::Vector2d position){
    bool collision = false; //collision checker
    int counter = 0; //polygon counter to tell which object has been collided with

    // Access every obstacle
    for (amp::Obstacle2D o : obstacles){
        std::vector<Eigen::Vector2d> v = o.verticesCCW();
        collision = true;

        //std::cout << "Next Object:" << std::endl;

        //Make a primitive for each obstacle
        for (int j = 0; j < v.size()-1; j++){
            Eigen::ParametrizedLine<double, 2> line(v[j], v[j+1] - v[j]);
            Eigen::Vector2d normal(line.direction()[1], -line.direction()[0]);

            //std::cout << "For edge: (" << v[j+1](0) << ", " << v[j+1](1) << ") and (" << v[j](0) << ", " << v[j](1) << ")" << std::endl;
            //std::cout << "The dot product of the normal vector (" << normal[0] << ", " << normal[1] << ") and the arbitrary vector (" << position[0]-v[j](0) << ", " << position[1]-v[j](1) << ") is " << normal.dot(position- v[j]) << std::endl;

            collision = collision && (normal.dot(position - v[j]) <= 0); 

            if (j == v.size()-2){
                Eigen::ParametrizedLine<double, 2> line(v[j+1], v[0] - v[j+1]);
                Eigen::Vector2d normal(line.direction()[1], -line.direction()[0]);

                //std::cout << "For edge: (" << v[j+1](0) << ", " << v[j+1](1) << ") and (" << v[j](0) << ", " << v[j](1) << ")" << std::endl;
                //std::cout << "The dot product of the normal vector (" << normal[0] << ", " << normal[1] << ") and the arbitrary vector (" << position[0]-v[j+1](0) << ", " << position[1]-v[j+1](1) << ") is " << normal.dot(position- v[j+1]) << std::endl;

                collision = collision && (normal.dot(position - v[j+1]) <= 0); 
            }
        }

        // finsh the loop
    

        if (collision){
            std::cout << "Collision!" << std::endl;
            return counter;
        }
        counter += 1;
    }
    return -1;
}

/// @brief Find the intersection point between all edges and return the closest one
/// @param q_last 
/// @param q_next 
/// @param obstacle 
/// @return intersection point on obstacle with path
Eigen::Vector2d closestPointOnObstacle(Eigen::Vector2d q_last, Eigen::Vector2d q_next, amp::Obstacle2D obstacle){
    // Declare variables
    std::vector<Eigen::Vector2d> v = obstacle.verticesCCW();
    std::vector<Eigen::Vector2d> closestPoint;
    using Line2 = Eigen::Hyperplane<double, 2>;

    // Initialize variables
    Line2 bugPath = Line2::Through(q_next, q_last); //Make a line in the path that the bug is going
    double closestDist = INT16_MAX;
    double closeDex;

    // Search for intersection points and return the closest one
    for (int j = 0; j < v.size(); j++){
        Line2 edge = Line2::Through(v[j], v[(j+1)%(v.size()-1)]); //Make a line in the path that the vertices are going
        closestPoint[j] = bugPath.intersection(edge);

        //Save the closest
        if ((closestPoint[j]-q_last).norm() < closestDist){
            closestDist = (closestPoint[j]-q_last).norm();
            closeDex = j;
        }
    }

    return closestPoint[closeDex];
}