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
/// @return A vertice list of all the points to follow in order
std::vector<Eigen::Vector2d> closestPointOnObstacle(Eigen::Vector2d q_last, Eigen::Vector2d q_next, amp::Obstacle2D obstacle){
    // Declare variables
    std::vector<Eigen::Vector2d> v = obstacle.verticesCCW();
    std::vector<Eigen::Vector2d> closestPoint;
    std::vector<Eigen::Vector2d> results;
    using Line2 = Eigen::Hyperplane<double, 2>;

    // Initialize variables
    Line2 bugPath = Line2::Through(q_next, q_last); //Make a line in the path that the bug is going
    double closestDist = INT16_MAX;
    int closeDex;


    // Search for intersection points and return the closest one
    for (int j = 0; j < v.size(); j++){
        Line2 edge = Line2::Through(v[j], v[(j+1)%(v.size())]); //Make a line in the path that the vertices are going
        closestPoint.push_back(bugPath.intersection(edge));
        //std::cout << j << ": " << ((j+1)%(v.size())) << ", v_j = (" <<  std::endl;
        
        //Save the closest point
        if ((closestPoint[j]-q_last).norm() < closestDist){
            closestDist = (closestPoint[j]-q_last).norm();
            closeDex = j;
        }
        
    }

    results.push_back(closestPoint[closeDex]);

    // Check the object that has been colided with for interecting obstacles and merge to a vector direction matrix for the bug to follow
        //See if any vertices overlap


    //Make a list of all vertices in the correct order starting from closeDex
    for (int j = 0; j < v.size(); j++){
        //Check if clostestPoint is also a vertex and don't add it to the list if it is
        //if (closestPoint[closeDex] == v[(j+closeDex)%(v.size())]){
        //    continue;
        //}

        results.push_back(v[(j+1+closeDex)%(v.size())]);
    }

    // Deubgging: write out all the vertices in the results vector to see if they are in the correct order
    std::cout << "Object Vertex Direction List: " << std::endl;  
    for (int j = 0; j < results.size(); j++){
        std::cout << "(" << results[j](0) << ", " << results[j](1) << ")" << std::endl;
    }

    //results.push_back(v[closeDex]);

    return results;
}

/// @brief 
/// @param obstaclePath 
/// @param q_goal 
/// @return shortest path from q_hit to q_leave on boundry of obstacle
std::vector<Eigen::Vector2d> findShortestPath(std::vector<Eigen::Vector2d> obstaclePath, Eigen::Vector2d q_goal){
    // Find the distance to the goal of all the vectors contained in the obstacle path and find the shortest one
    double shortestD;
    double currentD;
    int shortestDdex = 0;
    for (int j = 0; j < obstaclePath.size(); j++){
        currentD = (obstaclePath[j] - q_goal).squaredNorm();
        if (j == 0){
            shortestD = currentD;
        }else{
            if (currentD < shortestD){
                shortestD = currentD;
                shortestDdex = j;
            }
        }
    }

    // Split the obstacle path into two vectors in half using the shortest distance index as the seperator
    std::vector<Eigen::Vector2d> obstaclePath1;
    std::vector<Eigen::Vector2d> obstaclePath2;
    std::vector<Eigen::Vector2d> shortestObstaclePath;
    for (int j = 0; j < shortestDdex; j++){
        obstaclePath1.push_back(obstaclePath[j]);
    }
    for (int j = obstaclePath.size()-1; j >= shortestDdex; j--){
        obstaclePath2.push_back(obstaclePath[j]);
    }

    if (vertVecDistance(obstaclePath1) < vertVecDistance(obstaclePath2)){
        shortestObstaclePath = obstaclePath1;
    }else{
        shortestObstaclePath = obstaclePath2;
    }

    // Deubgging: write out all the vertices in the results vector to see if they are in the correct order
    std::cout << "Obstacle Path Direction List: " << std::endl;  
    for (int j = 0; j < obstaclePath.size(); j++){
        std::cout << "(" << obstaclePath[j](0) << ", " << obstaclePath[j](1) << ")" << std::endl;
    }
    std::cout << "Shortest Path Direction List: " << std::endl;  
    for (int j = 0; j < shortestObstaclePath.size(); j++){
        std::cout << "(" << shortestObstaclePath[j](0) << ", " << shortestObstaclePath[j](1) << ")" << std::endl;
    }

    return shortestObstaclePath;
}

// Returns the distance of the path
double pathDistance(amp::Path2D path){
    double distance = 0;
    for (int i = 0; i < path.waypoints.size()-1; i++){
        //print out the individual distances
        //std::cout << "The distance between (" << path.waypoints[i](0) << ", " << path.waypoints[i](1) << ") and (" << path.waypoints[i+1](0) << ", " << path.waypoints[i+1](1) << ") is: " << (path.waypoints[i+1] - path.waypoints[i]).norm() << std::endl;

        distance += (path.waypoints[i+1] - path.waypoints[i]).norm();
    }
    std::cout << "The path distance is: " << distance << std::endl;
    return distance;
}

// Returns the distance of the path
double vertVecDistance(std::vector<Eigen::Vector2d> path){
    double distance = 0;
    for (int i = 0; i < path.size()-1; i++){
        //print out the individual distances
        //std::cout << "The distance between (" << path.waypoints[i](0) << ", " << path.waypoints[i](1) << ") and (" << path.waypoints[i+1](0) << ", " << path.waypoints[i+1](1) << ") is: " << (path.waypoints[i+1] - path.waypoints[i]).norm() << std::endl;

        distance += (path[i+1] - path[i]).norm();
    }
    std::cout << "The path distance is: " << distance << std::endl;
    return distance;
}