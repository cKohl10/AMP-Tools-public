
#include "CSpace.h"
#include "HelpfulClass.h"

CSpace::CSpace(const amp::LinkManipulator2D& robot, const amp::Environment2D& enviroment, std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max) : amp::GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max){
    m_robot = &robot;
    m_enviroment = enviroment;
}

bool CSpace::inCollision(double x0, double x1) const{

    ManipulatorState state = {x0, x1};
    //Check if the end of each link is in colision, then subdivide the link and check again
    Eigen::Vector2d prevP = m_robot->getJointLocation(state, 0); //Previous joint location
    Eigen::Vector2d nextP; //Next joint location
    int subdivs = 1; //Number of subdivisions to check

    //Loops through each point in the robot arm
    for (int i = 0; i < m_robot->getLinkLengths().size(); i++){
        nextP = m_robot->getJointLocation(state, i+1);
        if (collisionDetected(prevP)){
            return true;
        } else {

            if (subdivide(prevP, nextP, subdivs)){
                return true;
            }
        }

        prevP = nextP;
    }
    //Check if the polygon is in collision
    return false;
}

/// @brief This builds a union of primitives for each obstacle and compares the location of the point to the union
/// @param position 
/// @return False for no collision or True for collision
bool CSpace::collisionDetected(Eigen::Vector2d position) const{
    bool collisionCCW = false; //collision checker
    bool collisionCW = false; //collision checker

    // Access every obstacle
    for (amp::Obstacle2D o : m_enviroment.obstacles){
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

//Subdivides until a specific number of subdivisions is reached. Checks for collisions on all subdivisions
bool CSpace::subdivide(Eigen::Vector2d b1, Eigen::Vector2d b2, int counter) const{
    Eigen::Vector2d mid = (b1+b2)/2.0;
    if (collisionDetected(mid)){
        return true;
    } else if (counter == 0){
        return false;
    } else {
        return subdivide(b1, mid, counter-1) || subdivide(mid, b2, counter-1);
    }
}
