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
     std::vector<bool> run = {false, true, false, true};

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
        std::cout << "##### Running Centralized Benchmarks #####" << std::endl;
        //This test will run the centralized planner 100 times for each number of agents
        //The results will be stored in the text file results.txt.
        //The results will store the number of agents, the number of nodes in the tree, the run time, and if the solution was valid

        //Open the file
        std::ifstream myfile_read;
        myfile_read.open ("Centralized_Benchmark.csv");
        if (!myfile_read.is_open()){
            std::cout << "Error opening file" << std::endl;
            return 0;
        }


        //Get the most recent number of agents and iteration
        //Get the last row of myfile
        std::string line;
        std::string last_line;
        while (std::getline(myfile_read, line)){
            last_line = line;
        }

        //get the first and second elements in the comma seperated line
        int i_max = 0;
        int m_max = 1;
        std::string added_line = "";
        if (last_line.size() > 0){
            std::string delimiter = ", ";
            std::string token = last_line.substr(0, last_line.find(delimiter));
            m_max = std::stoi(token);
            last_line.erase(0, last_line.find(delimiter) + delimiter.length());
            token = last_line.substr(0, last_line.find(delimiter));
            i_max = std::stoi(token);

            std::cout << "Save file found at m = " << m_max << " and i = " << i_max << std::endl;
        } else{
            std::cout << "No save file found" << std::endl;
            added_line = "Number of Agents, Iteration, Tree Size, Run Time, Valid Solution\n";
        }

        //Close the file
        myfile_read.close();

        //Open the file to write to
        std::ofstream myfile;
        myfile.open ("Centralized_Benchmark.csv", std::ios_base::app);
        if (!myfile.is_open()){
            std::cout << "Error opening file" << std::endl;
            return 0;
        }

        //Write the header to the file
        myfile << added_line;

        //Benchmarks for the centralized planner
        std::list<std::vector<double>> tree_sizes;
        std::list<std::vector<double>> run_times;
        std::vector<int> valid_solutions;

        //Set Hyperparameters
        double r = 0.5; //Step size;
        double p_goal = 0.05; //Probability of sampling the goal state;
        int n = 7500; //Number of samples;
        double epsilon = 0.5; //Termination radius;
        int m = m_max; //Max number of agents
        if (i_max == 100){
            i_max = 0;
            m++;
        }

        while (m <= 6){

            std::cout <<std::endl << "Running with " << m << " agents" << std::endl;
            std::cout << "Iteration: " << i_max << std::endl;
            std::vector<double> tree_size;
            std::vector<double> run_time;
            int valid = 0;

            for (int i = i_max; i < 100; i++){
                // Start time
                std::clock_t start = std::clock();

                //Call centralized motion planner
                MACentralized mac(r, p_goal, n, epsilon, m);

                //Plan paths for each agent 
                //amp::MultiAgentProblem2D problem = amp::HW8::getWorkspace1(m);
                //amp::MultiAgentPath2D ma_path = mac.plan(problem);
                //amp::HW8::check(ma_path, problem, true);
                amp::HW8::generateAndCheck(mac, true);

                // End time
                std::clock_t end = std::clock();

                //Set Benchmarks
                double elapsed = static_cast<double>(end - start) / (CLOCKS_PER_SEC*1000); //ms
                run_time.push_back(elapsed);
                tree_size.push_back(mac.getTreeSize()); //nodes
                //bool valid = amp::HW8::check(ma_path, problem, false);
                //valid = valid + amp::HW8::check(ma_path, problem, false);

                //Write the results to the file
                myfile << m << ", " << i+1 <<", " << mac.getTreeSize() << ", " << elapsed << ", " << valid << "\n";
                //std::cout << m_max << ", " << mac.getTreeSize() << ", " << elapsed << ", " << valid << std::endl;

                std::cout << "\033[A\33[2K\r";
                std::cout << "Progress: " << i+1 << "/100" << std::endl;
            }

            std::cout << "\033[A\33[2K\r";
            std::cout << "Valid Solutions: " << valid << "/100" << std::endl;

            m++;
            
            //Update the contents to the file every iteration
            myfile.flush();
        }

        //Close the file
        myfile.close();

        //Compute the average time and tree size
        // Average computation size box plot
    }

    // //Plotting results 
    // if (run[2]){
    //     //Open the centralized planner file 
    //     std::ifstream myfile_read;
    //     myfile_read.open ("Centralized_Benchmark.csv");
    //     if (!myfile_read.is_open()){
    //         std::cout << "Error opening file" << std::endl;
    //         return 0;
    //     }

    //     //Average computation and tree size box plot
    //     std::vector<std::string> num_agents = {"1", "2", "3", "4", "5", "6"};

    //     std::list<std::vector<double>> tree_sizes;
    //     std::list<std::vector<double>> run_times;

    //     //Get the data from the file, ignoring the header, tree size is the third element, run time is the fourth element
    //     std::string line;
    //     while(std::getline(myfile_read, line)){
    //         if (line.find("Number of Agents") == std::string::npos){
    //             std::string delimiter = ", ";
    //             std::string token = line.substr(0, line.find(delimiter));
    //             line.erase(0, line.find(delimiter) + delimiter.length());
    //             token = line.substr(0, line.find(delimiter));
    //             line.erase(0, line.find(delimiter) + delimiter.length());
    //             token = line.substr(0, line.find(delimiter));
    //             double tree_size = std::stod(token);
    //             line.erase(0, line.find(delimiter) + delimiter.length());
    //             token = line.substr(0, line.find(delimiter));
    //             double run_time = std::stod(token);

    //             tree_sizes.push_back({tree_size});
    //             run_times.push_back({run_time});
    //         }
    //     }
    

    //     amp::Visualizer::makeBoxPlot(tree_sizes, num_agents, "Centralized Multi Agent RRT - Tree Size", "Number of Agents", "Tree Size");

    // }

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

    if (run[3]){
        std::cout << "##### Running Decentralized Benchmarks #####" << std::endl;
        //This test will run the centralized planner 100 times for each number of agents
        //The results will be stored in the text file results.txt.
        //The results will store the number of agents, the number of nodes in the tree, the run time, and if the solution was valid

        //Open the file
        std::ifstream myfile_read;
        myfile_read.open ("Decentralized_Benchmark.csv");
        if (!myfile_read.is_open()){
            std::cout << "Error opening file" << std::endl;
            return 0;
        }


        //Get the most recent number of agents and iteration
        //Get the last row of myfile
        std::string line;
        std::string last_line;
        while (std::getline(myfile_read, line)){
            last_line = line;
        }

        //get the first and second elements in the comma seperated line
        int i_max = 0;
        int m_max = 1;
        std::string added_line = "";
        if (last_line.size() > 0){
            std::string delimiter = ", ";
            std::string token = last_line.substr(0, last_line.find(delimiter));
            m_max = std::stoi(token);
            last_line.erase(0, last_line.find(delimiter) + delimiter.length());
            token = last_line.substr(0, last_line.find(delimiter));
            i_max = std::stoi(token);

            std::cout << "Save file found at m = " << m_max << " and i = " << i_max << std::endl;
        } else{
            std::cout << "No save file found" << std::endl;
            added_line = "Number of Agents, Iteration, Tree Size, Run Time, Valid Solution\n";
        }

        //Close the file
        myfile_read.close();

        //Open the file to write to
        std::ofstream myfile;
        myfile.open ("Decentralized_Benchmark.csv", std::ios_base::app);
        if (!myfile.is_open()){
            std::cout << "Error opening file" << std::endl;
            return 0;
        }

        //Write the header to the file
        myfile << added_line;

        //Benchmarks for the centralized planner
        std::list<std::vector<double>> run_times;
        std::vector<int> valid_solutions;

        //Set Hyperparameters
        double r = 0.5; //Step size;
        double p_goal = 0.05; //Probability of sampling the goal state;
        int n = 7500; //Number of samples;
        double epsilon = 0.5; //Termination radius;
        int m = m_max; //Max number of agents
        if (i_max == 100){
            i_max = 0;
            m++;
        }

        while (m <= 6){

            std::cout <<std::endl << "Running with " << m << " agents" << std::endl;
            std::cout << "Iteration: " << i_max << std::endl;
            std::vector<double> run_time;
            int valid = 0;

            for (int i = i_max; i < 100; i++){
                // Start time
                std::clock_t start = std::clock();

                //Call centralized motion planner
                MADecentralized mad(r, p_goal, n, epsilon, m);

                //Plan paths for each agent 
                amp::MultiAgentProblem2D problem = amp::HW8::getWorkspace1(m);
                amp::MultiAgentPath2D ma_path = mad.plan(problem);
                //amp::HW8::check(ma_path, problem, true);

                // End time
                std::clock_t end = std::clock();

                //Set Benchmarks
                double elapsed = static_cast<double>(end - start) / (CLOCKS_PER_SEC*1000); //ms
                run_time.push_back(elapsed);
                //bool valid = amp::HW8::check(ma_path, problem, false);
                valid = valid + amp::HW8::check(ma_path, problem, false);

                //Write the results to the file
                myfile << m << ", " << i+1 <<", " << -1 << ", " << elapsed << ", " << valid << "\n";
                //std::cout << m_max << ", " << mac.getTreeSize() << ", " << elapsed << ", " << valid << std::endl;

                std::cout << "\033[A\33[2K\r";
                std::cout << "Progress: " << i+1 << "/100" << std::endl;
            }

            std::cout << "\033[A\33[2K\r";
            std::cout << "Valid Solutions: " << valid << "/100" << std::endl;

            m++;
            
            //Update the contents to the file every iteration
            myfile.flush();
        }

        //Close the file
        myfile.close();

        //Compute the average time and tree size
        // Average computation size box plot
    }
    
    
    //Show all the figures
    amp::Visualizer::showFigures();

    //Grade the homework
    amp::HW8::grade<MACentralized, MADecentralized>("carson.kohlbrenner@colorado.edu", argc, argv);

    return 0;
}



