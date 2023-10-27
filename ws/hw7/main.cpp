#include "AMPCore.h"
#include "hw/HW7.h"
#include "hw/HW5.h"
#include "ShareCore.h"


using namespace amp;

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    //Vector of bools to make it easier to run the right graphs
    std::vector<bool> run = {true, false};

    // ############ Exercise 1 ###############
    // Probablistic Roadmap
    if (run[0]){

        //Get Problem 
        amp::Problem2D problem = amp::HW5::getWorkspace1();

        //Set the bounds of the problem
        Eigen::Vector2d xbounds = {-1, 11};
        Eigen::Vector2d ybounds = {-3, 3};

        //Set the number of nodes to sample and the radius of the neighborhood
        int n = 100;
        double r = 1;

        //Construct the problem
        PRMAlgo2D prm_algo(xbounds, ybounds, n, r);

        //Plan the path
        prm_algo.plan(problem);
       
    }

    // ############ Exercise 2 ###############
    // RRT
    if (run[1]){
       
    }

    //amp::HW7::hint();

    //Show figures
    //Visualizer::showFigures();

    //amp::HW7::grade(prm_algo, rrt_algo, carson.kohlbrenner@colorado.edu, int argc, char** argv);
    return 0;
}