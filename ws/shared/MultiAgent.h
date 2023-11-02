#pragma once

#include "AMPCore.h"
#include "RRT.h"
#include "HelpfulClass.h"
#include "hw/HW8.h"

class MACentralized : public amp::CentralizedMultiAgentRRT {
    public:

        MACentralized();

        /// @brief Plans paths with GoalBiasedRRT for each disk agent 
        /// @param problem 
        /// @return 
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;
    
    private:
        double n; //number or RRT samples
        double r; //stepsize of rrt
        double p_goal; //probability of sampling goal
        double r_goal; //radius of goal

};