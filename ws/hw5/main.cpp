// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"
#include "hw/HW2.h"

// Include any custom headers you created in your workspace
#include "ShareCore.h"

using namespace amp;

int main(int argc, char** argv) {

    // Choose problems to run
    std::vector<bool> run = {true, true, true};
    
    
    if (run[0]){
        std::cout << "Running Problem 2A..." << std::endl;

        //################ For Workspace 2A ######################
        //  Attracitve Potential
        double zeta = 1;
        double d_star = 0.5;

        //  Repulsive Potential
        double Q_star = 2;
        double eta = 0.1;

        //  Gradient Descent
        double epsilon = 0.01;
        double stepSize = 0.01;
        int maxSteps = 10000;
        
        //  Range Detector
        int rays = 20;
        PotentialPlanner GDPlanner(zeta, d_star, eta, Q_star, epsilon, stepSize, maxSteps, rays);
        //########################################################

        // Get Workspaces
        Problem2D problem = HW5::getWorkspace1();

        // Call your algorithm on the problem
        amp::Path2D path = GDPlanner.plan(problem);

        //Return path distance
        std::cout << "Path Distance: " << pathDistance(path) << std::endl;

        // Check your path to make sure that it does not collide with the environment 
        bool success = HW5::check(path, problem);

        LOG("Found valid solution to workspace 1: " << (success ? "Yes!" : "No :("));

        // Visualize the path and environment
        Visualizer::makeFigure(problem, path);
    }

    if (run[1]){
        std::cout << "Running Problem 2B-1..." << std::endl;

        //################ For Workspace 2A ######################
        //  Attracitve Potential
        double zeta = 0.02;
        double d_star = 2;

        //  Repulsive Potential
        double Q_star = 10;
        double eta = 0.002;

        //  Gradient Descent
        double epsilon = 0.00001;
        double stepSize = 0.5;
        int maxSteps = 10000;

        //Initial Push - Needed for some workspaces
        Eigen::Vector2d push_direction = Eigen::Vector2d(1, 0);
        double push_magnitude = 0.0 /stepSize;
        
        //  Range Detector
        int rays = 100;
        PotentialPlanner GDPlanner(zeta, d_star, eta, Q_star, epsilon, stepSize, maxSteps, rays, push_direction, push_magnitude);
        GDPlanner.changeSensingMode();
        //########################################################

        // Get Workspaces
        Problem2D problem = HW2::getWorkspace1();

        // Call your algorithm on the problem
        amp::Path2D path = GDPlanner.plan(problem);

        //Return path distance
        std::cout << "Path Distance: " << pathDistance(path) << std::endl;

        // Check your path to make sure that it does not collide with the environment 
        bool success = HW5::check(path, problem);

        LOG("Found valid solution to workspace 2: " << (success ? "Yes!" : "No :("));

        // Visualize the path and environment
        Visualizer::makeFigure(problem, path);
    }

    if (run[2]){
        std::cout << "Running Problem 2B-2..." << std::endl;

        //################ For Workspace 2A ######################
        //  Attracitve Potential
        double zeta = 0.1;
        double d_star = 0.5;

        //  Repulsive Potential
        double Q_star = 1.5;
        double eta = 0.01;

        //  Gradient Descent
        double epsilon = 0.0001;
        double stepSize = 1;
        int maxSteps = 10000;
        
        //Initial Push - Needed for some workspaces
        Eigen::Vector2d push_direction = Eigen::Vector2d(0.0, 1).normalized();
        double push_magnitude = 0.0 / stepSize;
        
        //  Range Detector
        int rays = 100;
        PotentialPlanner GDPlanner(zeta, d_star, eta, Q_star, epsilon, stepSize, maxSteps, rays, push_direction, push_magnitude);
        GDPlanner.changeSensingMode();
        //########################################################

        // Get Workspaces
        Problem2D problem = HW2::getWorkspace2();

        // Call your algorithm on the problem
        amp::Path2D path = GDPlanner.plan(problem);

        //Return path distance
        std::cout << "Path Distance: " << pathDistance(path) << std::endl;

        // Check your path to make sure that it does not collide with the environment 
        bool success = HW5::check(path, problem);

        LOG("Found valid solution to workspace 3: " << (success ? "Yes!" : "No :("));

        // Visualize the path and environment
        Visualizer::makeFigure(problem, path);
    }

    Visualizer::showFigures();

    //HW2::grade(algo, "carson.kohlbrenner@colorado.edu", argc, argv);

    return 0;
}