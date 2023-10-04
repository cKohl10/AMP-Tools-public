// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "HelpfulClass.h"
#include "LinkMan.h"

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
    //amp::Visualizer::showFigures();

    //################# Problem 2 #################//
    std::cout << "Starting Problem 2: " << std::endl;
    std::vector<double> linkLengths = {1.0, 2.0, 3.0};
    ManipulatorState state = {0.0, 0.0, 0.0};

    LinkMan manipulator = LinkMan(linkLengths);
    amp::Visualizer::makeFigure(manipulator, state);
    amp::Visualizer::showFigures();

    //Get the joint location of end effector
    //Eigen::Vector2d jointLocation = manipulator.getJointLocation(state, 3);


    // Grade method
    //amp::HW4::grade<MyLinkManipulator>(constructor, "nonhuman.biologic@myspace.edu", argc, argv);
    return 0;
}