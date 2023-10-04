#include "LinkMan.h"

class LinkMan : public amp::LinkManipulator2D{
    public:
        Eigen::Vector2d getJointLocation(const ManipulatorState& state, uint32_t joint_index) const{

        }
        ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const{
            
        }
};