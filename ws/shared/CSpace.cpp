
#include "CSpace.h"

CSpace::CSpace(const amp::LinkManipulator2D& robot, const amp::Environment2D& environment, std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max) : amp::GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max){
    m_robot = &robot;
    m_environment = environment;
}

std::pair<std::size_t, std::size_t> CSpace::getCellFromPoint(double x0, double x1) const {

    //Calculate the step size for each dimension
    double x0_step = (m_x0_bounds.second - m_x0_bounds.first)/size().first;
    double x1_step = (m_x1_bounds.second - m_x1_bounds.first)/size().second;

    //Calculate the cell indices and floor them
    std::size_t i = (x0 - m_x0_bounds.first)/x0_step;
    std::size_t j = (x1 - m_x1_bounds.first)/x1_step;

    //Debuging:
    // std::cout << std::endl << "New getCellFromPoint() function called!" << std::endl;
    // std::cout << "x0: " << x0 << " x1: " << x1 << std::endl;
    // std::cout << "x0_step: " << x0_step << " x1_step: " << x1_step << std::endl;
    // std::cout << "m_x0_bounds.first: " << m_x0_bounds.first << " m_x0_bounds.second: " << m_x0_bounds.second << std::endl;
    // std::cout << "m_x1_bounds.first: " << m_x1_bounds.first << " m_x1_bounds.second: " << m_x1_bounds.second << std::endl;
    // std::cout << "i: " << i << " j: " << j << std::endl << std::endl;

    return std::make_pair(i, j);
}


//Checks the full link manipulator for collisions
bool CSpace::inCollision(double x0, double x1) const{

    //############# Line Collision Detection #############//
    amp::ManipulatorState2Link state = {x0, x1};
    std::vector<amp::Polygon> o = m_environment.obstacles;

    // Start by finding all the joint vertices of the robot
    Eigen::Vector2d v0 = m_robot->getJointLocation(state,0);
    Eigen::Vector2d v1 = m_robot->getJointLocation(state,1);
    Eigen::Vector2d v2 = m_robot->getJointLocation(state,2);

    // Next, find if any of the vertices are in collision with any of the obstacles
    for (int i = 0; i < o.size(); i++){

        // get vertices current obstacle
        std::vector<Eigen::Vector2d> obsVertices = o[i].verticesCCW();

        // now, for each vertice, check if it collides with the line between v0 and v1 and v1 and v2
        for (int j = 0; j < obsVertices.size(); j++){
            // check if the line between v0 and v1 and v1 and v2 collides with the line between obsVertices[j] and obsVertices[j+1]
            if (intersect(v0,v1,obsVertices[j%obsVertices.size()],obsVertices[(j+1)%obsVertices.size()]) || intersect(v1,v2,obsVertices[j%obsVertices.size()],obsVertices[(j+1)%obsVertices.size()])){
                return true;
            }
        }
    }
    return false;

    /*
    //############# Subdivision Collision Detection #############//
    //Check if the end of each link is in colision, then subdivide the link and check again
    Eigen::Vector2d prevP = m_robot->getJointLocation(state, 0); //Previous joint location
    Eigen::Vector2d nextP; //Next joint location
    int subdivs = 10; //Number of subdivisions to check

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
    return false;*/
}

/// @brief This builds a union of primitives for each obstacle and compares the location of the point to the union
/// @param position 
/// @return False for no collision or True for collision
bool CSpace::collisionDetected(Eigen::Vector2d position) const{
    bool collisionCCW = false; //collision checker
    bool collisionCW = false; //collision checker

    // Access every obstacle
    for (amp::Obstacle2D o : m_environment.obstacles){
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

//Determines if there is an intersection point between two lines
//Credit: https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
bool CSpace::intersect(Eigen::Vector2d p1, Eigen::Vector2d q1, Eigen::Vector2d p2, Eigen::Vector2d q2) const{
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
