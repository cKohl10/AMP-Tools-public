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