#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW7.h"
#include "hw/HW5.h"
#include "hw/HW8.h"
#include "RRT.h"
#include <ctime>
//#include "ShareCore.h"

int main(int argc, char** argv) {
    std::vector<bool> run = {true, false};
    amp::MultiAgentProblem2D problem = amp::HW8::getWorkspace1();

    // ############ Exercise 1 ###############
    // Multi-agent RRT
    if (run[0]){

        //Call centralized motion planner
        MACentralized mac;

        //Plan paths for each agent 
        amp::MultiAgentPath2D ma_path = mac.plan(problem);

        amp::Visualizer::makeFigure(problem, ma_path);
    }

    if (run[1]){

    }

    //Show all the figures
    amp::Visualizer::showFigures();

    //Grade the homework

    return 0;
}