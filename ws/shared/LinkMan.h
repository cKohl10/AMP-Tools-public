#pragma once

#include "AMPCore.h"
#include "HelpfulClass.h"

/// @brief Vector of angles (radians) for each joint. The size of the vector should match the 
/// number of links (and hence joints) of the manipulator
//using ManipulatorState = std::vector<double>;

/// @brief List of manipulator states in chronological order
//using ManipulatorTrajectory = std::list<ManipulatorState>;

class LinkMan : public amp::LinkManipulator2D{
    public:
        //Class contructors
        LinkMan();
        LinkMan(const std::vector<double>& link_lengths);
        LinkMan(const Eigen::Vector2d& base_location, const std::vector<double>& link_lengths);

        Eigen::Vector2d getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const;
        amp::ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const;

    private:
        Eigen::MatrixXd Tmatrix(double angle, double a) const;
};