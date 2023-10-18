#include "HelpfulClass.h"



Eigen::Vector2d rotateVec2d(Eigen::Vector2d &vec, double angle){
    Eigen::Matrix2d rot;
    rot << cos(angle), -sin(angle), sin(angle), cos(angle);
    vec = rot * vec;
    return vec;
}

//Returns the angle of a vector in world coordinates
double vecAngle(Eigen::Vector2d v1, Eigen::Vector2d v2){
    Eigen::Vector2d vNorm = (v2-v1).normalized();
    //vNorm = rotateVec2d(vNorm, M_PI/2.0);
    
    //print the vNorm indices
    double angle = atan2(vNorm.y(), vNorm.x());
    angle = (angle < 0) ? angle + 2*M_PI : angle;
    //std::cout << "vNorm: (" << vNorm.x() << ", " << vNorm.y() << "), angle of " << angle*(180/M_PI) << std::endl;


    return angle;
}

/// @brief Reorder a set of vertices to start from its bottom left most point
/// @param av 
/// @return 
std::vector<Eigen::Vector2d> minStartReorder(std::vector<Eigen::Vector2d> &av){
    //Find the minimum vertice
    int amindex = 0;
    for (int i = 1; i < av.size(); i++){
        
        if (av[i].y() < av[amindex].y() ){
            amindex = i;
        }
        
        //anglePrev = angleNext;
    }

    //push the front to the back of the vertex vector until the minimum is at the front
    for (int i = 0; i < amindex; i++){
        av.push_back(av.front());
        av.erase(av.begin());
    }
    return av;
}


/// @brief Rotates an object by the reference points
/// @param poly 
/// @param angle 
/// @return 
amp::Polygon rotatePolygon(amp::Polygon &poly, double angle){
    std::vector<Eigen::Vector2d> v = poly.verticesCCW();
    for (int i = 0; i < v.size(); i++){
        Eigen::Vector2d vec = v[i]-v[0];
        v[i] = rotateVec2d(vec, angle) + v[0];
    }
    return amp::Polygon(v);
}

/// @brief Tranlsates a polygon to a target point based on the reference point of first vertice listed
/// @param A 
/// @param targ 
/// @return 
amp::Polygon translatePolygon(amp::Polygon &A, Eigen::Vector2d targ){
    //Find the distance from the first vertice to the target
    std::vector<Eigen::Vector2d> v = A.verticesCCW();
    Eigen::Vector2d dist = targ - v.front();

    //Translate the polygon by the distance
    for (int i = 0; i < v.size(); i++){
        v[i] += dist;
    }

    return amp::Polygon(v);
}

/// @brief Takes the mikownski sum of two polygons
/// @param a Robot Polygon
/// @param b Obstacl Polygon
/// @return a new polygon that is the minkowski sum of a and b
amp::Polygon minkowskiSum(amp::Polygon robot, amp::Polygon b) {
    //Invert the robot polygon
    Eigen::Vector2d ref = robot.verticesCCW().front(); //Reference point of the robot
    amp::Polygon a = invertPolygon(robot);
    //amp::Polygon a = robot;

    //Reorder vertices of a and b to start at minimum x+y value
    std::vector<Eigen::Vector2d> av = a.verticesCCW();
    std::vector<Eigen::Vector2d> bv = b.verticesCCW();

    //Reverse the order of the robots vertices for minkowski sum
    //std::reverse(av.begin(), av.end());

    minStartReorder(av);
    minStartReorder(bv);

    //Run through minkowski sum algorithm
    std::vector<Eigen::Vector2d> cObsVertices;
    
    int i = 0;
    int j = 0;
    while(i < av.size()+1 && j < bv.size()+1){
        //Convert inverted vertice back to normal when indexing
        //Eigen::Vector2d reala = ref - (av[i%av.size()]-ref);

        //Push into final list of vertices
        Eigen::Vector2d input = av[i%av.size()]+bv[j%bv.size()];
        cObsVertices.push_back(input);
        std::cout << "Added vertice a" << i << " + b" << j << " --> (" << input.x() << ", " << input.y() << ")" << std::endl;

        //Print out what is being compared
        double anglea = vecAngle(av[(i)%av.size()], av[(i+1)%av.size()]);
        double angleb = vecAngle(bv[(j)%bv.size()], bv[(j+1)%bv.size()]);

        //std::cout << "Polygon a: " << std::endl;
        //std::cout << "(" << av[i].x() << ", " << av[i].y() << ") to (" << av[(i+1)%av.size()].x() << ", " << av[(i+1)%av.size()].y() << "), angle of " << anglea*(180/M_PI) << std::endl;
        //std::cout << "Polygon b: " << std::endl;
        //std::cout << "(" << bv[j].x() << ", " << bv[j].y() << ") to (" << bv[(j+1)%bv.size()].x() << ", " << bv[(j+1)%bv.size()].y() << "), angle of " << angleb*(180/M_PI) << std::endl << std::endl;

        if (anglea < angleb){
            i++;
        } else if (anglea > angleb){
            j++;
        } else{
            i++;
            j++;
        }
    }

    amp::Polygon cObs = amp::Polygon(cObsVertices);
    return cObs;

}

/// @brief Prints out all the vertices in a polygon
/// @param a 
void printPolygonVertices(amp::Polygon a){
    std::vector<Eigen::Vector2d> av = a.verticesCCW();

    std::cout << "####### Front of Vector #######" << std::endl;
    for (int i = 0; i < av.size(); i++){
        std::cout << "(" << av[i][0] << ", " << av[i][1] << ")" << std::endl;
    }
    std::cout << "####### End of Vector #######" << std::endl;
}

/// @brief Inverts a polygons vertices about the reference point of the first vertice
/// @param a 
/// @return 
amp::Polygon invertPolygon(amp::Polygon a){
    std::vector<Eigen::Vector2d> av = a.verticesCCW();
    for (int i = 0; i < av.size(); i++){
        av[i] = av[0] - (av[i]-av[0]);
    }
    return amp::Polygon(av);
}

//credit to https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
bool intersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& q1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q2){
    double x1 = p1.x();
    double y1 = p1.y();
    double x2 = q1.x();
    double y2 = q1.y();
    double x3 = p2.x();
    double y3 = p2.y();
    double x4 = q2.x();
    double y4 = q2.y();

    // Check if none of the lines are of length 0
    if ((x1 == x2 && y1 == y2) || (x3 == x4 && y3 == y4)) {
        return false;
    }
    double denom = ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
    // Lines are parallel
    if (denom == 0) {
        return false;
    }
    double ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3))/denom;
    double ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3))/denom;
    // is the intersection along the segments
    if (ua < 0 || ua > 1 || ub < 0 || ub > 1) {
        return false;
    }
    return true; // is intersection
}

// Returns the distance of the path
double pathDistance(amp::Path2D path){
    double distance = 0;
    for (int i = 0; i < path.waypoints.size()-1; i++){
        //print out the individual distances
        //std::cout << "The distance between (" << path.waypoints[i](0) << ", " << path.waypoints[i](1) << ") and (" << path.waypoints[i+1](0) << ", " << path.waypoints[i+1](1) << ") is: " << (path.waypoints[i+1] - path.waypoints[i]).norm() << std::endl;

        distance += (path.waypoints[i+1] - path.waypoints[i]).norm();
    }
    //std::cout << "The path distance is: " << distance << std::endl;
    return distance;
}

/// @brief This builds a union of primitives for each obstacle and compares the location of the point to the union
/// @param position 
/// @return False for no collision or True for collision
bool collisionDetectedPoint(std::vector<amp::Obstacle2D> obstacles, Eigen::Vector2d position){
    bool collisionCCW = false; //collision checker
    bool collisionCW = false; //collision checker

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

            //Eigen::ParametrizedLine<double, 2> line(v[j], v[(j+1)%v.size()] - v[j]);
            //Eigen::Vector2d normal(line.direction()[1], -line.direction()[0]);

            Eigen::Vector2d edgeVec = v[(j+1)%v.size()] - v[j];
            Eigen::Vector2d posVec = position - v[j];
            //Eigen::Vector2d normal = rotateVec2d(vec, M_PI/2.0);

            //std::cout << "For edge: (" << v[j+1](0) << ", " << v[j+1](1) << ") and (" << v[j](0) << ", " << v[j](1) << ")" << std::endl;
            //std::cout << "The dot product of the normal vector (" << normal[0] << ", " << normal[1] << ") and the arbitrary vector (" << position[0]-v[j](0) << ", " << position[1]-v[j](1) << ") is " << normal.dot(position- v[j]) << std::endl;

            collisionCCW = collisionCCW && (edgeVec.dot(posVec) <= 0); 
            collisionCW = collisionCW && (edgeVec.dot(posVec) > 0); 
        }

        //std::cout << std::endl;

        // finsh the loop
    

        if (collisionCCW || collisionCW){
            //std::cout << "Collision!" << std::endl;
            return true;
        }
    }

    return false;
}
