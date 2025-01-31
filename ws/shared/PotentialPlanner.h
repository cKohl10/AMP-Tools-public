#pragma once

#include "AMPCore.h"
#include "ShareCore.h"
#include "hw/HW5.h"

struct GDVars{
    public:
        GDVars(double zeta, double d_star, double eta, double Q_star, double epsilon, double stepSize, int maxSteps, int rays, bool rawScanning);

    //############# HYPER PARAMETERS #############
        //  Attracitve Potential
        double zeta; // Scalar for attractive potential
        double d_star; //Distance from the goal where attractive potential switches between quadratic and conic
        
        //  Repulsive Potential
        double eta; // Scalar for repulsive potential
        double Q_star; //Distance from the obstacle where repulsive potential is in effect
        
        //  Range Detector
        RangeDetector sensor;

        //  Gradient Descent
        double epsilon; //Threshold for the potential function exit condition
        double stepSize; //Step size for gradient descent
        int maxSteps; //Maximum number of steps for gradient descent
        double goalRange; //Distance from the goal where the robot is considered to have reached the goal
        bool doRawScanning;

        int rays; //Number of rays the sensor has
        //############################################
};

class PotentialPlanner : public amp::GDAlgorithm{
    public:
        PotentialPlanner();
        PotentialPlanner(double zeta, double d_star, double eta, double Q_star, double epsilon, double stepSize, int maxSteps, int rays); 
        PotentialPlanner(double zeta, double d_star, double eta, double Q_star, double epsilon, double stepSize, int maxSteps, int rays, Eigen::Vector2d push_direction, double push_magnitude);
        PotentialPlanner(std::vector<GDVars> varSets);
        amp::Path2D plan(const amp::Problem2D& problem);
        void changeSensingMode();
        void changePlan();

    private:
        //############# HYPER PARAMETERS #############
        //  Attracitve Potential
        double zeta; // Scalar for attractive potential
        double d_star; //Distance from the goal where attractive potential switches between quadratic and conic
        
        //  Repulsive Potential
        double eta; // Scalar for repulsive potential
        double Q_star; //Distance from the obstacle where repulsive potential is in effect
        
        //  Range Detector
        RangeDetector sensor;

        //  Gradient Descent
        double epsilon; //Threshold for the potential function exit condition
        double stepSize; //Step size for gradient descent
        int maxSteps; //Maximum number of steps for gradient descent
        double goalRange; //Distance from the goal where the robot is considered to have reached the goal
        bool doRawScanning;

        //  Push Potential
        Eigen::Vector2d push_direction; //Direction of the push
        double push_magnitude; //Magnitude of the push

        //  Gradient Descent Variables
        std::vector<GDVars> varSets;
        int setIndex;
        //############################################

        
        /**
         * Calculates the attractive potential for a given configuration q, with respect to a goal configuration q_goal.
         * The attractive potential is a measure of how much the robot is "attracted" to the goal configuration.
         *
         * @param q The current configuration of the robot.
         * @param q_goal The goal configuration that the robot is trying to reach.
         *
         * @return The attractive potential at the given configuration q.
         */
        Eigen::Vector2d attractivePotential(const Eigen::Vector2d& q, const Eigen::Vector2d& q_goal);


        /**
         * Calculates the repulsive potential for a given point q with respect to a set of obstacles.
         * The repulsive potential is used to avoid obstacles while planning a path to the goal.
         *
         * @param q The point for which the repulsive potential is to be calculated.
         * @param q_goal The goal point towards which the path is being planned.
         * @param obstacles A vector of polygons representing the obstacles in the detected environment.
         *
         * @return The repulsive potential at the given point q.
         */
        Eigen::Vector2d repulsivePotential(const Eigen::Vector2d& q, const Eigen::Vector2d& q_goal, const std::vector<amp::Polygon>& obstacles);
};