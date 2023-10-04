// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "HelpfulClass.h"
#include "LinkMan.h"
#include "CSpace.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());


//################# Problem 1 #################//

    std::cout << "Starting Problem 1: " << std::endl;
    // Workspace obstacles and robot set to be visualized
    std::vector<amp::Polygon> polygons;

    // Declare triangle obstacle and robot
    amp::Polygon robot = amp::HW4::getEx1TriangleObstacle();
    amp::Polygon triObstacle = amp::HW4::getEx1TriangleObstacle();

    std::vector<double> heights;
    int steps = 12;
    for (int k = 0; k < steps; k++) {

        //Create the minkowski sum
        amp::Polygon cObs = minkowskiSum(robot, triObstacle);

        //Rotate the robot
        robot = rotatePolygon(robot, (2*M_PI)/steps);
        //robot = translatePolygon(robot, Eigen::Vector2d(2.0*k, 4.0*k));

        //Add in the z level for each program
        heights.push_back(0.2*k);
        polygons.push_back(cObs);
    } 

    //Create 3d figure from visualizer class
    std::vector<amp::Polygon> polygonsP1a;
    polygonsP1a.push_back(polygons[0]);
    //amp::Visualizer::makeFigure(polygonsP1a);
    //amp::Visualizer::makeFigure(polygons, heights);

//################# Problem 2 #################//
    std::cout << "Starting Problem 2a: " << std::endl;
    std::vector<double> linkLengths2a = {0.5, 1, 0.5};
    ManipulatorState state2a = {M_PI/6.0, M_PI/3.0, (7.0*M_PI)/4.0};

    LinkMan manipulator2a = LinkMan(linkLengths2a);
    //amp::Visualizer::makeFigure(manipulator2a, state2a);

    std::cout << "Starting Problem 2b: " << std::endl;
    std::vector<double> linkLengths2b = {1.0, 0.5, 1.0};

    LinkMan manipulator2b = LinkMan(linkLengths2b);
    ManipulatorState state2b = manipulator2b.getConfigurationFromIK(Eigen::Vector2d(2.0, 0.0));
    //print results of state2b
    std::cout << "State 2b: " << state2b[0] << ", " << state2b[1] << ", " << state2b[2] << std::endl;

    //amp::Visualizer::makeFigure(manipulator2b, state2b);


//################# Problem 3 #################//
    std::cout << "Starting Problem 3a: " << std::endl;

    //Create the environment
    amp::Environment2D env3a = amp::HW4::getEx3Workspace1();
    amp::Environment2D env3b = amp::HW4::getEx3Workspace2();
    amp::Environment2D env3c = amp::HW4::getEx3Workspace3();

    //Create the link manipulator
    std::vector<double> linkLengths3a = {1.0, 1.0};
    LinkMan manipulator3a = LinkMan(linkLengths3a);

    //Create the C-Space
    std::size_t x0_cells = 100; 
    std::size_t x1_cells = 100; 
    double x0_min = 0; 
    double x0_max = 2*M_PI; 
    double x1_min = 0; 
    double x1_max = 2*M_PI; 
    CSpace cspace3a = CSpace(manipulator3a, env3a, x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max);

    //Create the figure
    //Visualize the environment and link manipulator
    amp::Visualizer::makeFigure(env3a, manipulator3a, manipulator3a.getConfigurationFromIK(Eigen::Vector2d(1.0, 1.0)));
    amp::Visualizer::makeFigure(env3b, manipulator3a, manipulator3a.getConfigurationFromIK(Eigen::Vector2d(1.0, 1.0)));
    amp::Visualizer::makeFigure(env3c, manipulator3a, manipulator3a.getConfigurationFromIK(Eigen::Vector2d(1.0, 1.0)));
    amp::Visualizer::makeFigure(cspace3a);

    //Display all figures that have been made
    amp::Visualizer::showFigures();

    // Grade method
    //amp::HW4::grade<MyLinkManipulator>(constructor, "nonhuman.biologic@myspace.edu", argc, argv);
    return 0;
}