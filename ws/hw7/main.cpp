#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW7.h"
#include "hw/HW5.h"
#include "PRM.h"
#include "RRT.h"
#include <ctime>
//#include "ShareCore.h"


using namespace amp;

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    //Vector of bools to make it easier to run the right graphs
    std::vector<bool> run = {true, false, false, false};
    std::vector<amp::Problem2D> problems = {amp::HW5::getWorkspace1(), amp::HW2::getWorkspace1(), amp::HW2::getWorkspace2()};
    std::vector<std::string> problem_names = {"HW5 EX1", "HW2 EX1", "HW2 EX2"};

// ############ Exercise 1ai ###############
    // Probablistic Roadmap
    if (run[0]){

        for (amp::Problem2D p : problems){
            //Get Problem 
            amp::Problem2D problem = p;

            //Set the bounds of the problem
            Eigen::Vector2d xbounds = {problem.x_min, problem.x_max};
            Eigen::Vector2d ybounds = {problem.y_min, problem.y_max};

            //Set the number of nodes to sample and the radius of the neighborhood
            int n = 400;
            double r = 2;

            //Construct the problem
            //PRMAlgo2D prm_algo(xbounds, ybounds, n, r);
            PRMAlgo2D prm_algo;

            //Plan the path
            amp::Path2D path = prm_algo.planWithFigure(problem);

            //amp::Visualizer::makeFigure(problem, path);
        }
        
        amp::Visualizer::showFigures();
    }

// ############ Exercise 1aii ###############
    // Varying the number of nodes and radius
    if (run[1]){

        for (int i = 0; i < problems.size(); i++){
            //Get Problem 
            amp::Problem2D problem = problems[i];

            //Set the bounds of the problem
            Eigen::Vector2d xbounds = {problem.x_min, problem.x_max};
            Eigen::Vector2d ybounds = {problem.y_min, problem.y_max};

            std::list<std::vector<double>> hpSets = {{200, 0.5}, {200, 1}, {200, 1.5}, {200, 2}, {500, 0.5}, {500, 1}, {500, 1.5}, {500, 2}};
            std::vector<std::string> hpSetsList = {"200, 0.5", "200, 1", "200, 1.5", "200, 2", "500, 0.5", "500, 1", "500, 1.5", "500, 2"};
            std::list<std::vector<double>> success_rates;
            std::list<std::vector<double>> path_lengths;
            std::list<std::vector<double>> run_times;

            int j = 1;

            for (auto& x : hpSets){
                double success_count = 0;

                std::vector<double> paths;
                std::vector<double> times;

                //loop 100 times
                for (int i = 0; i < 100; i++){
                    // Start time
                    std::clock_t start = std::clock();

                    //Set the number of nodes to sample and the radius of the neighborhood
                    int n = x[0];
                    double r = x[1];

                    //Construct the problem
                    PRMAlgo2D prm_algo(xbounds, ybounds, n, r);

                    //Plan the path
                    amp::Path2D path = prm_algo.plan(problem);
                    // End time
                    std::clock_t end = std::clock();
                    //Count how many success are in each benchmark
                    if (path.waypoints.size() > 0){
                        success_count++;
                        paths.push_back(pathDistance(path));
                        double elapsed = static_cast<double>(end - start) / CLOCKS_PER_SEC;
                        times.push_back(elapsed);

                    }
                }

                std::cout << "Batch " << i+1 << "/" << problems.size() << " Trail " << j << "/" << hpSets.size() << " Completed: " << success_count << "/100 Paths Found" << std::endl;
                j++;

                //Add success rate to vector
                success_rates.push_back({success_count});
                path_lengths.push_back(paths);
                run_times.push_back(times);

            }

            //Make a bar graph
            //amp::Visualizer::makeBarGraph(success_rates, hpSetsList, problem_names[i] + " Benchmark", "Number of Nodes, Radius", "Success Rate");

        }
       
       amp::Visualizer::showFigures();
    }

// ############ Exercise 2a ###############
    // RRT
    if (run[2]){
       for (amp::Problem2D p : problems){
            //Get Problem 
            amp::Problem2D problem = p;

            //Set the bounds of the problem
            Eigen::Vector2d xbounds = {problem.x_min, problem.x_max};
            Eigen::Vector2d ybounds = {problem.y_min, problem.y_max};

            //Set the number of nodes to sample and the radius of the neighborhood
            int n = 5000;
            double r = 0.5;
            double p_goal = 0.05;
            double epsilon = 0.25;

            //Construct the problem
            RRTAlgo2D rrt_algo(r, p_goal, n, epsilon, xbounds, ybounds);

            //Plan the path
            amp::Path2D path = rrt_algo.planWithFigure(problem);
            for (auto& x : path.waypoints){
                std::cout << x[0] << ", " << x[1] << std::endl;
            }


            //amp::Visualizer::makeFigure(problem, path);
        }
        
        amp::Visualizer::showFigures();
    }

// ############ Exercise 2b ###############
    // Varying the number of nodes and radius
    if (run[3]){

        //Comparison Metrics
        std::vector<double> success_rates;

        for (int i = 0; i < problems.size(); i++){
            //Get Problem 
            amp::Problem2D problem = problems[i];

            //Set the bounds of the problem
            Eigen::Vector2d xbounds = {problem.x_min, problem.x_max};
            Eigen::Vector2d ybounds = {problem.y_min, problem.y_max};

            //Set the number of nodes to sample and the radius of the neighborhood
            int n = 5000;
            double r = 0.5;
            double p_goal = 0.05;
            double epsilon = 0.25;

            //Construct the problem
            RRTAlgo2D rrt_algo(r, p_goal, n, epsilon, xbounds, ybounds);

            int success_count = 0;

            //loop 100 times
            for (int i = 0; i < 100; i++){

                //Plan the path
                amp::Path2D path = rrt_algo.plan(problem);

                //Count how many success are in each benchmark
                if (path.waypoints.size() > 0){
                    success_count++;
                }
            }

            std::cout << "Environment " << i+1 << "/" << problems.size() << " Completed: " << success_count << "/100 Paths Found" << std::endl;

            //Add success rate to vector
            success_rates.push_back(success_count);

        }
       
        //Make a bar graph
        amp::Visualizer::makeBarGraph(success_rates, problem_names, "RRT Benchmark", "Environment", "Success Rate");
        amp::Visualizer::showFigures();
    }

    //amp::HW7::hint();

    //Show figures
    //Visualizer::showFigures();

    amp::HW7::grade<PRMAlgo2D, RRTAlgo2D>("carson.kohlbrenner@colorado.edu", argc, argv);
    return 0;
}