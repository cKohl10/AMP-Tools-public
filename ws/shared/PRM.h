#pragma once

//#include "ShareCore.h"
#include "AMPCore.h"
#include "hw/HW7.h"
#include "hw/HW6.h"
#include "HelpfulClass.h"
#include "Astar.h"

class PointCollisionChecker2D : amp::ConfigurationSpace {
    public:
        PointCollisionChecker2D(const Eigen::VectorXd& lower_bounds, const Eigen::VectorXd& upper_bounds, const amp::Environment2D* environment);

        // Returns false if the point is not in collision
        virtual bool inCollision(const Eigen::VectorXd& state) const override;

        const amp::Environment2D* getEnvironment();

    private:
        const amp::Environment2D *m_environment;

};

class GenericPRM {
    public:
        /// @brief Solves a path for a generic N-dimension prm problem
        /// @param init_state 
        /// @param goal_state 
        /// @param collision_checker 
        /// @return 
        amp::Path planxd(const Eigen::VectorXd& init_state, const Eigen::VectorXd& goal_state, std::shared_ptr<PointCollisionChecker2D> collision_checker);

        void sampleMap(std::map<amp::Node, Eigen::VectorXd>& node_map, std::shared_ptr<PointCollisionChecker2D> collision_checker);

        //For Depth first search
        bool traverseChildren(amp::Node currNode, amp::Node goalNode);

        //amp::Path planxd(const Eigen::VectorXd& init_state, const Eigen::VectorXd& goal_state, PointCollisionChecker* collision_checker);

    public:
        std::vector<Eigen::Vector2d> bounds;
        int n; //Number of samples
        double r; //Radius of neighborhood

        std::map<amp::Node, Eigen::VectorXd> node_map;
        amp::Graph<double> graph;

        // A* Variables
        amp::LookupSearchHeuristic heuristic;

        // Depth First Search Variables
        std::map<amp::Node, bool> processed_nodes;
        std::vector<amp::Node> node_path;
};

class PRMAlgo2D : public amp::PRM2D, public GenericPRM {
    public:
        PRMAlgo2D();

        //Initialize with the Cspace bounds
        PRMAlgo2D(Eigen::Vector2d xbounds, Eigen::Vector2d ybounds);

        //Initialize with the Cspace bounds and the number of nodes to sample and the radius of the neighborhood
        PRMAlgo2D(Eigen::Vector2d xbounds, Eigen::Vector2d ybounds, int n, double r);

        //Plan a path in 2d using the members in problem
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;
        amp::Path2D planWithFigure(const amp::Problem2D& problem);

    private:
        std::map<amp::Node, Eigen::Vector2d> node_map2D;
        



};