#pragma once

#include "AMPCore.h"

/// @brief Vector of angles (radians) for each joint. The size of the vector should match the 
/// number of links (and hence joints) of the manipulator
using ManipulatorState = std::vector<double>;

/// @brief List of manipulator states in chronological order
using ManipulatorTrajectory = std::list<ManipulatorState>;

class LinkMan : public amp::LinkManipulator2D{
    public:
        Eigen::Vector2d getJointLocation(const ManipulatorState& state, uint32_t joint_index) const;
        ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const;
};