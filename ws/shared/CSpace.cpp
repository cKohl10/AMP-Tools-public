
#include "CSpace.h"

CSpace::CSpace(LinkMan robot, amp::Environment2D enviroment, std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max) : amp::GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max){
    m_robot = robot;
    m_enviroment = enviroment;
}

bool CSpace::inCollision(double x0, double x1) const{

    ManipulatorState state = {x0, x1};
    Eigen::Vector2d position = m_robot.getJointLocation(state, 1);
    if (collisionDetected(position)){
        return true;
    }
    //Check if the polygon is in collision
    return false;
}

/// @brief This builds a union of primitives for each obstacle and compares the location of the point to the union
/// @param position 
/// @return False for no collision or True for collision
bool CSpace::collisionDetected(Eigen::Vector2d position){
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
            std::cout << "Collision!" << std::endl;
            return true;
        }
    }

    return false;
}