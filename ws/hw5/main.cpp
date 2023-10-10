// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"
#include "hw/HW2.h"

// Include any custom headers you created in your workspace
#include "ShareCore.h"

using namespace amp;

int main(int argc, char** argv) {

    // Use WO1 from Exercise 2
    Problem2D problem = HW5::getWorkspace1();

    // Use WO2 from Exercise 2
    /*
    Problem2D problem = HW2::getWorkspace2();
    */

    // Make a random environment spec, edit properties about it such as the number of obstacles
    /*
    Random2DEnvironmentSpecification spec;
    spec.max_obstacle_region_radius = 5.0;
    spec.n_obstacles = 2;
    spec.path_clearance = 0.01;
    spec.d_sep = 0.01;

    //Randomly generate the environment;
    Problem2D problem = EnvironmentTools::generateRandom(spec); // Random environment
    */
    //problem = fixOverlappingPolygons(problem);

    // Declare your algorithm object 
    PotentialPlanner algo;
    
    {
        // Call your algorithm on the problem
        amp::Path2D path = algo.plan(problem);

        // Check your path to make sure that it does not collide with the environment 
        bool success = HW5::check(path, problem);

        LOG("Found valid solution to workspace 1: " << (success ? "Yes!" : "No :("));

        // Visualize the path and environment
        Visualizer::makeFigure(problem, path);
    }

    Visualizer::showFigures();

    //HW2::grade(algo, "carson.kohlbrenner@colorado.edu", argc, argv);

    return 0;
}