#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW7.h"
#include "hw/HW5.h"
#include "hw/HW8.h"
#include "RRT.h"
#include <ctime>
#include <iostream>
#include <fstream>
//#include "ShareCore.h"

int main(int argc, char** argv) {
    std::vector<bool> run = {false, true, false};

    // ############ Exercise 1 ###############
    // Multi-agent RRT
    if (run[0]){

        //Set Hyperparameters
        double r = 0.5; //Step size;
        double p_goal = 0.05; //Probability of sampling the goal state;
        int n = 7500; //Number of samples;
        double epsilon = 0.5; //Termination radius;
        int m = 2; //Max number of agents

        while (m <= 6){

            std::cout <<std::endl << "Running with " << m << " agents" << std::endl;

            //Call centralized motion planner
            MACentralized mac(r, p_goal, n, epsilon, m);

            //Plan paths for each agent 
            amp::MultiAgentProblem2D problem = amp::HW8::getWorkspace1(m);
            amp::MultiAgentPath2D ma_path = mac.plan(problem);
            amp::HW8::check(ma_path, problem, true);

            amp::Visualizer::makeFigure(problem, ma_path);

            m++;
        }
    }

    if (run[1]){
        //This test will run the centralized planner 100 times for each number of agents
        //The results will be stored in the text file results.txt.
        //The results will store the number of agents, the number of nodes in the tree, the run time, and if the solution was valid

        //Open the file
        // std::ofstream myfile;
        // myfile.open ("results.txt");
        // if (!myfile.is_open()){
        //     std::cout << "Error opening file" << std::endl;
        //     return 0;
        // }
        
        //Write to the file with the headers
        myfile << "Number of Agents, Tree Size, Run Time, Valid Solution\n";
        myfile.flush();

        //Benchmarks for the centralized planner
        std::list<std::vector<double>> tree_sizes;
        std::list<std::vector<double>> run_times;
        std::list<std::vector<bool>> valid_solutions;

        //Set Hyperparameters
        double r = 0.5; //Step size;
        double p_goal = 0.05; //Probability of sampling the goal state;
        int n = 7500; //Number of samples;
        double epsilon = 0.5; //Termination radius;
        int m = 1; //Max number of agents

        while (m <= 6){

            std::cout <<std::endl << "Running with " << m<< " agents" << std::endl;
            std::vector<double> tree_size;
            std::vector<double> run_time;
            std::vector<bool> valid_solution;

            for (int i = 0; i < 100; i++){
                // Start time
                std::clock_t start = std::clock();

                //Call centralized motion planner
                MACentralized mac(r, p_goal, n, epsilon, m);

                //Plan paths for each agent 
                amp::MultiAgentProblem2D problem = amp::HW8::getWorkspace1(m);
                amp::MultiAgentPath2D ma_path = mac.plan(problem);
                //amp::HW8::check(ma_path, problem, true);

                // End time
                std::clock_t end = std::clock();

                //Set Benchmarks
                double elapsed = static_cast<double>(end - start) / (CLOCKS_PER_SEC*1000); //ms
                run_time.push_back(elapsed);
                tree_size.push_back(mac.getTreeSize()); //nodes
                //bool valid = amp::HW8::check(ma_path, problem, false);
                bool valid = (ma_path.agent_paths.size() > 0);
                valid_solution.push_back(valid);

                //Write the results to the file
                myfile << m_max << ", " << mac.getTreeSize() << ", " << elapsed << ", " << valid << "\n";
                //std::cout << m_max << ", " << mac.getTreeSize() << ", " << elapsed << ", " << valid << std::endl;
            }

            m++;
            
            //Update the contents to the file every iteration
            myfile.flush();
        }

        //Close the file
        myfile.close();

        //Compute the average time and tree size
        // Average computation size box plot
        std::vector<std::string> num_agents = {"1", "2", "3", "4", "5", "6"};
        amp::Visualizer::makeBoxPlot(tree_sizes, num_agents, "Centralized Multi Agent RRT - Tree Size", "Number of Agents", "Tree Size");
    }

    // ############ Exercise 2 ###############
    // Multi-agent RRT decentralized
    if (run[2]){

        //Set Hyperparameters
        double r = 0.5; //Step size;
        double p_goal = 0.05; //Probability of sampling the goal state;
        int n = 7500; //Number of samples;
        double epsilon = 0.5; //Termination radius;
        int m = 2; //Max number of agents

        while (m <= 6){

            std::cout <<std::endl << "Running with " << m << " agents" << std::endl;

            //Call centralized motion planner
            MADecentralized mad(r, p_goal, n, epsilon, m);

            //Plan paths for each agent 
            amp::MultiAgentProblem2D problem = amp::HW8::getWorkspace1(m);
            amp::MultiAgentPath2D ma_path = mad.plan(problem);
            amp::HW8::check(ma_path, problem, true);

            amp::Visualizer::makeFigure(problem, ma_path);

            m++;
        }
    }

    
    
    //Show all the figures
    amp::Visualizer::showFigures();

    //Grade the homework
    amp::HW8::grade<MACentralized, MADecentralized>("carson.kohlbrenner@colorado.edu", argc, argv);

    return 0;
}



