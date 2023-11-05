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

        //Set Hyperparameters
        double r = 0.5; //Step size;
        double p_goal = 0.05; //Probability of sampling the goal state;
        int n = 7500; //Number of samples;
        double epsilon = 0.25; //Termination radius;
        int m_max = 2; //Max number of agents

        //Call centralized motion planner
        MACentralized mac(r, p_goal, n, epsilon, m_max);

        //Plan paths for each agent 
        amp::MultiAgentPath2D ma_path = mac.plan(problem);

        amp::Visualizer::makeFigure(problem, ma_path);
    }

    if (run[1]){

        //Benchmarks for the centralized planner
        std::list<std::vector<double>> tree_sizes;
        std::list<std::vector<double>> run_times;
    }

    //Show all the figures
    amp::Visualizer::showFigures();

    //Grade the homework

    return 0;
}