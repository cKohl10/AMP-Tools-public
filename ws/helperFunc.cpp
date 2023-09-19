#include "helperFunc.h"


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
std::vector<int> collisionDetected(std::vector<amp::Obstacle2D> obstacles, Eigen::Vector2d position){
    bool collisionCCW = false; //collision checker
    bool collisionCW = false; //collision checker
    int counter = 0; //polygon counter to tell which object has been collided with
    std::vector<int> collisionDexs; //List of all the objects that have been collided with

    // Access every obstacle
    for (amp::Obstacle2D o : obstacles){
        std::vector<Eigen::Vector2d> v = o.verticesCCW();
        
        //print object vertices v
        //std::cout << std::endl << "Object " << counter << " vertices: " << v.size() <<std::endl;

        collisionCCW = true;
        collisionCW = true;

        //std::cout << "Next Object:" << std::endl;

        //Make a primitive for each obstacle
        for (int j = 0; j < v.size(); j++){
            if ((v[(j+1)%v.size()] - v[j]).norm() == 0.0){
                //print if same vertices
                //std::cout << "Same vertices" << std::endl;
                continue;
            }
            Eigen::ParametrizedLine<double, 2> line(v[j], v[(j+1)%v.size()] - v[j]);
            Eigen::Vector2d normal(line.direction()[1], -line.direction()[0]);

            //std::cout << "For edge: (" << v[j+1](0) << ", " << v[j+1](1) << ") and (" << v[j](0) << ", " << v[j](1) << ")" << std::endl;
            //std::cout << "The dot product of the normal vector (" << normal[0] << ", " << normal[1] << ") and the arbitrary vector (" << position[0]-v[j](0) << ", " << position[1]-v[j](1) << ") is " << normal.dot(position- v[j]) << std::endl;

            collisionCCW = collisionCCW && (normal.dot(position - v[j]) <= 0); 
            collisionCW = collisionCW && (normal.dot(position - v[j]) > 0); 
        }

        //std::cout << std::endl;

        // finsh the loop
    

        if (collisionCCW || collisionCW){
            //std::cout << "Collision!" << std::endl;
            counter;
            collisionDexs.push_back(counter);
        }
        counter += 1;
    }

    if (collisionDexs.size() == 0){
        collisionDexs.push_back(-1);
    }
    return collisionDexs;
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

    results.push_back(closestPoint[closeDex]); //qHit

    // Check the object that has been colided with for interecting obstacles and merge to a vector direction matrix for the bug to follow
        //See if any vertices overlap


    //Make a list of all vertices in the correct order starting from closeDex
    for (int j = 0; j < v.size(); j++){
        //Check if clostestPoint is also a vertex and don't add it to the list if it is
        //if (closestPoint[closeDex] == v[(j+closeDex)%(v.size())]){
        //    continue;
        //}

        results.push_back(v[(j+1+closeDex)%(v.size())]); //All vertices in counter clockwise order
    }

    results.push_back(closestPoint[closeDex]); //qHit again

    ////////////////// NOTE //////////////////////
    // The output can be reversed to change directions 

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
    /*
    std::cout << "Obstacle Path Direction List Found: " << std::endl;  
    for (int j = 0; j < obstaclePath.size(); j++){
        std::cout << "(" << obstaclePath[j](0) << ", " << obstaclePath[j](1) << ")" << std::endl;
    }
    std::cout << "Shortest Path Direction List Found: " << std::endl;  
    for (int j = 0; j < shortestObstaclePath.size(); j++){
        std::cout << "(" << shortestObstaclePath[j](0) << ", " << shortestObstaclePath[j](1) << ")" << std::endl;
    }
    */

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

//Finds the center point for each polygon in the workspace and scales vertices from the center
amp::Problem2D expandPolygons(amp::Problem2D problem, double expansionFactor){
    amp::Problem2D newProblem = problem; //Make a copy of the problem to edit
    Eigen::Vector2d center; //Center point of the polygon
    int counter = 0; //Polygon counter

    //Access every obstacle
    for (amp::Obstacle2D o : problem.obstacles){

        center = Eigen::Vector2d::Zero(); //Reset the center point

        //Find the center point of the polygon
        for (Eigen::Vector2d v : o.verticesCCW()){
            center += v;
        }

        //Average the center point
        center /= o.verticesCCW().size();

        //Print the center point location
        std::cout << "Center point: (" << center(0) << ", " << center(1) << ")" << std::endl;

        //Scale the vertices from the center point
        for (int i = 0; i < o.verticesCCW().size(); i++){
            o.verticesCCW()[i] = o.verticesCCW()[i] + (o.verticesCCW()[i] - center).normalized()*expansionFactor;
        }

        //Save the expanded vertices
        newProblem.obstacles[counter] = o;
        counter++;
    }

    return newProblem;
}

/// @brief This builds a union of primitives for each obstacle and compares the location of each vertice for overlaps 
/// @param obstacles 
/// @param position 
/// @return false for no collision, or the object number in the list that was collided with
bool polygonOverlap(amp::Obstacle2D obstacle1, amp::Obstacle2D obstacle2){
    bool collision = false; //collision checker
    
    std::vector<amp::Obstacle2D> obstaclesVec; //List of both obstacles
    obstaclesVec.push_back(obstacle1);
    obstaclesVec.push_back(obstacle2);

    // Access every obstacle
    for (int k = 0; k < obstaclesVec.size(); k++){

        //Set the vertices to test against as v and the vertices to test as v2
        std::vector<Eigen::Vector2d> v = obstaclesVec[k%2].verticesCCW();
        std::vector<Eigen::Vector2d> v2 = obstaclesVec[(k+1)%2].verticesCCW();
            
        for (int i = 0; i < v2.size(); i++){
            
            collision = true;
            Eigen::Vector2d position = v2[i]; //This is the vertice that will be tested for collision next

            //Make a primitive for each obstacle
            for (int j = 0; j < v.size(); j++){
                Eigen::ParametrizedLine<double, 2> line(v[j], v[(j+1)%v.size()] - v[j]);
                Eigen::Vector2d normal(line.direction()[1], -line.direction()[0]);

                //Checks every normal and only collides if direction vector dot with vertice is all <= 0
                collision = collision && (normal.dot(position - v[j]) <= 0); 

                //print out poistion vector and normal vector
                //std::cout << "The dot product of the normal vector (" << normal[0] << ", " << normal[1] << ") and the arbitrary vector (" << position[0]-v[j](0) << ", " << position[1]-v[j](1) << ") is " << normal.dot(position- v[j]) << std::endl;
            }

            // finsh the loop
            if (collision){
                //std::cout << "Object overlap detected!" << std::endl;
                return true;
            }
        }
    }
    return false;
}

/// @brief Takes in two parameterized lines and returns the angle between them
/// @param line1 
/// @param line2 
/// @return angle between 0 to pi
double angleBetweenLines(const Eigen::ParametrizedLine<double, 2>& line1, const Eigen::ParametrizedLine<double, 2>& line2) {
    double dotProduct = line1.direction().dot(line2.direction());
    double angle = std::acos(dotProduct);
    return angle;
}