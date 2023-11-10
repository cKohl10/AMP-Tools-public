#pragma once
#include "AMPCore.h"
#include "HelpfulClass.h"
#include "PRM.h"
#include "hw/HW8.h"

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
        RRTAlgo2D();

        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        amp::Path2D planWithFigure(const amp::Problem2D& problem);

        amp::Path planxd(const amp::Problem2D& problem);

        //amp::Path GoalBiasPlanXd(const Eigen::VectorXd& init_state, const Eigen::VectorXd& goal_state, PointCollisionChecker2D* collision_checker);
        amp::Node nearestNeighbor(const Eigen::VectorXd& q_rand);

        //######## Multi Agent RRT #########
        //amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem);

        //bool isSystemInGoal(std::vector<Eigen::VectorXd> q, const amp::MultiAgentProblem2D& problem);


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

class MACentralized : public amp::CentralizedMultiAgentRRT {
    public:
        MACentralized();

        MACentralized(double r, double p_goal, int n, double epsilon, int m);

        //######## Multi Agent Centralized RRT #########
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;

        bool isSystemInGoal(std::vector<Eigen::Vector2d> q, std::vector<Eigen::Vector2d> q_goal);

        //Splits a matrix with all states into a vector of 2D state positions for each circular robot
        std::vector<Eigen::Vector2d> decompose2D(Eigen::VectorXd q);

        //combines a vector of 2D state positions for each circular robot to a single state matrix
        Eigen::VectorXd compose2D(std::vector<Eigen::Vector2d> q);

        //Finds the closest node in the tree to the random node
        amp::Node nearestNeighbor(const std::vector<Eigen::Vector2d>& q_rand);

        //Checks robot collisions and collisions with other robots for valid paths
        bool isSubpathCollisionFree(const std::vector<Eigen::Vector2d>& q_near, const std::vector<Eigen::Vector2d>& q_new, const std::vector<amp::Obstacle2D>& obstacles);

        //Backs out a path for a given final node
        std::vector<amp::Path2D> backoutPath(amp::Node final_node);

        //Prints the state of the node
        void printState(std::vector<Eigen::Vector2d> state, std::string name);

        //Gets the number of nodes in the tree
        int getTreeSize();

    private:
        std::map<amp::Node, Eigen::VectorXd> node_map;
        std::map<amp::Node, std::vector<Eigen::Vector2d>> ma2d_node_map;
        amp::Graph<double> graph;

        // Hyperparameters
        double r; //Step size;
        double p_goal; //Probability of sampling the goal state;
        int n; //Number of samples;
        double epsilon; //Termination radius;
        //int m_max; //Maximum number of agents to process

        //Problem Variables
        double radius; //Radius of the circular agent
        std::vector<double> radii; //Radius of the circular agents
        int m; //Number of agents
        std::vector<Eigen::Vector2d> bounds; //Problem bounds

};

class MADecentralized : public amp::DecentralizedMultiAgentRRT {
    public:
        MADecentralized();

        MADecentralized(double r, double p_goal, int n, double epsilon, int m);

        //######## Multi Agent Deentralized RRT #########
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;

        amp::Path2D GoalBiasedRRT(const Eigen::Vector2d& init_state, const Eigen::Vector2d& q_goal, std::vector<amp::Obstacle2D> obstacles);

        //Checks robot collisions and collisions with other robots for valid paths
        bool isSubpathCollisionFree(const Eigen::Vector2d& q_near, const Eigen::Vector2d& q_new, const std::vector<amp::Obstacle2D>& obstacles, int time);

        //Find the nearest neighbor
        amp::Node nearestNeighbor(const Eigen::Vector2d& q_rand);

    private:
        std::map<amp::Node, Eigen::Vector2d> node_map;
        std::map<amp::Node, int> time_map;
        std::map<int, std::vector<Eigen::Vector2d>> time_of_others_map;
        //amp::Graph<double> graph;

        // Hyperparameters
        double r; //Step size;
        double p_goal; //Probability of sampling the goal state;
        int n; //Number of samples;
        double epsilon; //Termination radius;
        //int m_max; //Maximum number of agents to process

        //Problem Variables
        double radius; //Radius of the circular agent
        int m; //Number of agents
        std::vector<Eigen::Vector2d> bounds; //Problem bounds
};