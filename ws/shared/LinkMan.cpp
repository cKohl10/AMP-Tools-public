#include "LinkMan.h"

/// @brief Vector of angles (radians) for each joint. The size of the vector should match the 
/// number of links (and hence joints) of the manipulator
using ManipulatorState = std::vector<double>;


LinkMan::LinkMan() : amp::LinkManipulator2D(){

}

LinkMan::LinkMan(const std::vector<double>& link_lengths) : amp::LinkManipulator2D(link_lengths){
    m_link_lengths = link_lengths;
}

LinkMan::LinkMan(const Eigen::Vector2d& base_location, const std::vector<double>& link_lengths) : amp::LinkManipulator2D(base_location, link_lengths){
    m_link_lengths = link_lengths;
    m_base_location = base_location;
}

/// @brief Get the location of the nth joint using the current link attributes using Forward Kinematics
/// @param state Joint angle state (radians). Must have size() == nLinks()
/// @param joint_index Joint index in order of base to end effector 
/// (joint_index = 0 should return the base location, joint_index = nLinks() should return the end effector location)
/// @return Joint coordinate
Eigen::Vector2d LinkMan::getJointLocation(const ManipulatorState& state, uint32_t joint_index) const{

    //Make transform to first joint
    Eigen::MatrixXd T;
    T = Tmatrix(state[0], 0);

    //Make transform to rest of the intermediate joints
    for (int k = 1; k < joint_index; k++){
        T = T*Tmatrix(state[k], m_link_lengths[k-1]);
    }   

    //Make transform to end effector
    if (joint_index > 0){
        T = T*Tmatrix(0, m_link_lengths[joint_index-1]);
    }

    //Calculate transformation
    Eigen::MatrixXd finalLocMat = T*Eigen::Vector3d(m_base_location.x(), m_base_location.y(), 1);

    return Eigen::Vector2d(finalLocMat(0), finalLocMat(1));
}


/// @brief Set the configuration (link attributes) give an end effector location using Inverse Kinematics
/// @param end_effector_location End effector coordinate
/// @return Joint angle state (radians) in increasing joint index order. Must have size() ==nLinks()
ManipulatorState LinkMan::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const{

    //####### Assumptions #######
    // - The angle that the third link approaches the given location is defined by writst angle w
    // - Two possible states remain, one state is chosen by the signs in the IK equations and the other one is removed
    // - The first link is always at the origin

    //####### IK Equations for 3DOF #######
    // This is using the equations derived from HW3
    double w = M_PI; //Wrist Angle: Rad
    double x = end_effector_location.x(); //End Effector X: m
    double y = end_effector_location.y(); //End Effector Y: m
    double a1 = m_link_lengths[0]; //Link 1 Length: m    
    double a2 = m_link_lengths[1]; //Link 2 Length: m
    double a3 = m_link_lengths[2]; //Link 3 Length: m

    //Calculate middle angle first
    double t2 = acos(( pow(x+a3*cos(w), 2) + pow(y+a3*sin(w), 2) - pow(a1, 2) - pow(a2, 2) )/(2*a1*a2));

    //Check if valid. Returns true if NaN
    if (t2 != t2){
        std::cout << "Invalid IK" << std::endl;
        return {0, 0, 0};
    }

    double t1 = atan2((y+a3*sin(w)), (x+a3*cos(w))) - atan2(( a2*sin(t2) ),( a1+a2*cos(t2) ));
    double t3 = (M_PI+w) - t1 - t2;

    ManipulatorState state = {t1, t2, t3};

    return state;
}

Eigen::MatrixXd LinkMan::Tmatrix(double angle, double a) const{
    Eigen::MatrixXd T(3,3);
    T << cos(angle), -sin(angle), a,
         sin(angle), cos(angle), 0,
         0, 0, 1;
    return T;
}
