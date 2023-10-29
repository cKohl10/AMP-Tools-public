#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW7.h"
#include "hw/HW5.h"
#include "PRM.h"
#include "RRT.h"
//#include "ShareCore.h"


using namespace amp;

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    //Vector of bools to make it easier to run the right graphs
    std::vector<bool> run = {false, false, true};
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
            int n = 200;
            double r = 1;

            //Construct the problem
            PRMAlgo2D prm_algo(xbounds, ybounds, n, r);

            //Plan the path
            amp::Path2D path = prm_algo.planWithFigure(problem);

            amp::Visualizer::makeFigure(problem, path);
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
            std::vector<double> success_rates;

            int j = 1;

            for (auto& x : hpSets){
                int success_count = 0;

                //loop 100 times
                for (int i = 0; i < 100; i++){
                    //Set the number of nodes to sample and the radius of the neighborhood
                    int n = x[0];
                    double r = x[1];

                    //Construct the problem
                    PRMAlgo2D prm_algo(xbounds, ybounds, n, r);

                    //Plan the path
                    amp::Path2D path = prm_algo.plan(problem);

                    //Count how many success are in each benchmark
                    if (path.waypoints.size() > 0){
                        success_count++;
                    }
                }

                std::cout << "Batch " << i+1 << "/" << problems.size() << " Trail " << j << "/" << hpSets.size() << " Completed: " << success_count << "/100 Paths Found" << std::endl;
                j++;

                //Add success rate to vector
                success_rates.push_back(success_count);
            }

            //Make a bar graph
            amp::Visualizer::makeBarGraph(success_rates, hpSetsList, problem_names[i] + " Benchmark", "Number of Nodes, Radius", "Success Rate");

        }
       
       amp::Visualizer::showFigures();
    }

    // ############ Exercise 2 ###############
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

            amp::Visualizer::makeFigure(problem, path);
        }
        
        amp::Visualizer::showFigures();
    }

    //amp::HW7::hint();

    //Show figures
    //Visualizer::showFigures();

    //amp::HW7::grade(prm_algo, rrt_algo, carson.kohlbrenner@colorado.edu, int argc, char** argv);
    return 0;
}