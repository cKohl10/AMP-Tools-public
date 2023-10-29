#pragma once
#include "AMPCore.h"
#include "HelpfulClass.h"
#include "PRM.h"

class RRTAlgo2D : amp::GoalBiasRRT2D{
    public:
    
        /// @brief 
        /// @param r step size 
        /// @param p_goal probability of sampling the goal state
        /// @param n number of samples
        /// @param epsilon termination radius
        /// @param xbounds 
        /// @param ybounds 
        RRTAlgo2D(double r, double p_goal, int n, double epsilon, Eigen::Vector2d xbounds, Eigen::Vector2d ybounds);

        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        amp::Path2D planWithFigure(const amp::Problem2D& problem);

        amp::Path planxd(const amp::Problem2D& problem);

        //amp::Path GoalBiasPlanXd(const Eigen::VectorXd& init_state, const Eigen::VectorXd& goal_state, PointCollisionChecker2D* collision_checker);
        amp::Node nearestNeighbor(const Eigen::VectorXd& q_rand);


    private:
        std::map<amp::Node, Eigen::VectorXd> node_map;
        std::map<amp::Node, Eigen::Vector2d> node_map2D;
        amp::Graph<double> graph;

        // Hyperparameters
        double r; //Step size;
        double p_goal; //Probability of sampling the goal state;
        int n; //Number of samples;
        double epsilon; //Termination radius;
        std::vector<Eigen::Vector2d> bounds;
};