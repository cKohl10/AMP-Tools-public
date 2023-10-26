#include "AMPCore.h"
#include "hw/HW7.h"
#include "ShareCore.h"


using namespace amp;

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    //Vector of bools to make it easier to run the right graphs
    std::vector<bool> run = {true, false};

    // ############ Exercise 1 ###############
    // Probablistic Roadmap
    if (run[0]){
       
    }

    // ############ Exercise 2 ###############
    // RRT
    if (run[1]){
       
    }

    //Show figures
    Visualizer::showFigures();

    //amp::HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("carson.kohlbrenner@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple("hey therre"), std::make_tuple());
    return 0;
}