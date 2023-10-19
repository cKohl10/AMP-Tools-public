#include "LinkMan.h"

/// @brief Vector of angles (radians) for each joint. The size of the vector should match the 
/// number of links (and hence joints) of the manipulator


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
Eigen::Vector2d LinkMan::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const{

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
amp::ManipulatorState LinkMan::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const{
    std::vector<double> state;
    //####### Assumptions #######
    // - The angle that the third link approaches the given location is defined by writst angle w
    // - Two possible states remain, one state is chosen by the signs in the IK equations and the other one is removed
    // - The first link is always at the origin

//####### IK Equations for 1DOF #######
    if (m_link_lengths.size() == 1){
        std::cout << "1DOF IK not implemented" << std::endl;    
    }
//####### IK Equations for 2DOF #######
    if (m_link_lengths.size() == 2){
        double x = end_effector_location.x(); //End Effector X: m
        double y = end_effector_location.y(); //End Effector Y: m
        double a1 = m_link_lengths[0]; //Link 1 Length: m    
        double a2 = m_link_lengths[1]; //Link 2 Length: m
        double a3 = m_link_lengths[2]; //Link 3 Length: m
        double t2 = 0;
        t2= acos(( pow(x, 2) + pow(y, 2) - pow(a1, 2) - pow(a2, 2) )/(2*a1*a2));   
        double t1 = atan2((y), (x)) - atan2(( a2*sin(t2) ),( a1+a2*cos(t2) ));
        state = {t1, t2};
    }
//####### IK Equations for 3DOF #######
    if (m_link_lengths.size() == 3){
        // This is using the equations derived from HW3
        double w = 0; //Wrist Angle: Rad
        double x = end_effector_location.x(); //End Effector X: m
        double y = end_effector_location.y(); //End Effector Y: m
        double a1 = m_link_lengths[0]; //Link 1 Length: m    
        double a2 = m_link_lengths[1]; //Link 2 Length: m
        double a3 = m_link_lengths[2]; //Link 3 Length: m
        double t2 = 0;

        bool valid = false;

        while(w<(2*M_PI) && !valid){
            //Calculate middle angle first
            t2= acos(( pow(x+a3*cos(w), 2) + pow(y+a3*sin(w), 2) - pow(a1, 2) - pow(a2, 2) )/(2*a1*a2));
            std::cout << "t2 = " << t2 << std::endl;

            //Check if valid. Returns true if NaN
            if (t2 != t2){
                std::cout << "When w = " << w << ", no IK solution found" << std::endl;
            } else{
                std::cout << "3DOF solution when w = " << w << std::endl;
                valid = true;
                continue;
            }

            w += (2*M_PI)/2048.0;
        }

        if (!valid){
            //Try the singlularity case
            w = vecAngle(m_base_location, end_effector_location);
            t2= acos(( pow(x+a3*cos(w), 2) + pow(y+a3*sin(w), 2) - pow(a1, 2) - pow(a2, 2) )/(2*a1*a2));

            if (t2 != t2){
                std::cout << "No valid IK solution found" << std::endl;
                return amp::ManipulatorState();
            } else{
                std::cout << "3DOF solution at singluarity case when w = " << w << std::endl;
            }
        }

        double t1 = atan2((y+a3*sin(w)), (x+a3*cos(w))) - atan2(( a2*sin(t2) ),( a1+a2*cos(t2) ));
        double t3 = (M_PI+w) - t1 - t2;

        state = {t1, t2, t3};
        std::cout << "Valid Solution: Angles-->(" << t1 << ", " << t2 << ", " << t3 << ")" << std::endl;
    }

    //populate the state vector into a ManipulatorState object
    amp::ManipulatorState state2(state.size());
    for (int i = 0; i < state.size(); i++){
        state2[i] = state[i];
    }
    return state2;
}

Eigen::MatrixXd LinkMan::Tmatrix(double angle, double a) const{
    Eigen::MatrixXd T(3,3);
    T << cos(angle), -sin(angle), a,
         sin(angle), cos(angle), 0,
         0, 0, 1;
    return T;
}

