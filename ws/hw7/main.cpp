#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW7.h"
#include "hw/HW5.h"
#include "PRM.h"
//#include "ShareCore.h"


using namespace amp;

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    //Vector of bools to make it easier to run the right graphs
    std::vector<bool> run = {true, true, true};

    amp::Visualizer::makeFigure(amp::HW2::getWorkspace1());
    amp::Visualizer::makeFigure(amp::HW2::getWorkspace2());

// ############ Exercise 1ai ###############
    // Probablistic Roadmap
    if (run[0]){

        //Get Problem 
        amp::Problem2D problem = amp::HW5::getWorkspace1();

        //Set the bounds of the problem
        Eigen::Vector2d xbounds = {-1, 11};
        Eigen::Vector2d ybounds = {-3, 3};

        //Set the number of nodes to sample and the radius of the neighborhood
        int n = 200;
        double r = 1;

        //Construct the problem
        PRMAlgo2D prm_algo(xbounds, ybounds, n, r);

        //Plan the path
        amp::Path2D path = prm_algo.planWithFigure(problem);

        amp::Visualizer::makeFigure(problem, path);
       
    }

// ############ Exercise 1aii ###############
    // Varying the number of nodes and radius
    if (run[1]){

        //Get Problem 
        amp::Problem2D problem = amp::HW5::getWorkspace1();

        //Set the bounds of the problem
        Eigen::Vector2d xbounds = {-1, 11};
        Eigen::Vector2d ybounds = {-3, 3};

        std::list<std::vector<double>> hpSets = {{200, 0.5}, {200, 1}, {200, 1.5}, {200, 2}, {500, 0.5}, {500, 1}, {500, 1.5}, {500, 2}};
        std::vector<std::string> hpSetsList = {"200, 0.5", "200, 1", "200, 1.5", "200, 2", "500, 0.5", "500, 1", "500, 1.5", "500, 2"};
        std::vector<double> success_rates;

        for (auto& x : hpSets){
            int success_count = 0;

            std::cout << std::endl << std::endl << "#################### TRAIL " << x[0] << ", " << x[1] << " ####################" << std::endl << std::endl;

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

            //Add success rate to vector
            success_rates.push_back(success_count);
        }

        //Make a bar graph
        amp::Visualizer::makeBarGraph(success_rates, hpSetsList, "HW 5 Ex. 2 Benchmark", "Number of Nodes, Radius", "Success Rate");
       
    }

// ############ Exercise 1bii ###############
    // Varying the number of nodes and radius
    if (run[1]){

        //Get Problem 
        amp::Problem2D problem = amp::HW2::getWorkspace1();

        //Set the bounds of the problem
        Eigen::Vector2d xbounds = {-1, 11};
        Eigen::Vector2d ybounds = {-3, 3};

        std::list<std::vector<double>> hpSets = {{200, 0.5}, {200, 1}, {200, 1.5}, {200, 2}, {500, 0.5}, {500, 1}, {500, 1.5}, {500, 2}};
        std::vector<std::string> hpSetsList = {"200, 0.5", "200, 1", "200, 1.5", "200, 2", "500, 0.5", "500, 1", "500, 1.5", "500, 2"};
        std::vector<double> success_rates;

        for (auto& x : hpSets){
            int success_count = 0;

            std::cout << std::endl << std::endl << "#################### TRAIL " << x[0] << ", " << x[1] << " ####################" << std::endl << std::endl;

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

            //Add success rate to vector
            success_rates.push_back(success_count);
        }

        //Make a bar graph
        amp::Visualizer::makeBarGraph(success_rates, hpSetsList, "HW 5 Ex. 2 Benchmark", "Number of Nodes, Radius", "Success Rate");
       
    }

    // ############ Exercise 2 ###############
    // RRT
    if (run[2]){
       
    }

    //amp::HW7::hint();

    //Show figures
    Visualizer::showFigures();

    //amp::HW7::grade(prm_algo, rrt_algo, carson.kohlbrenner@colorado.edu, int argc, char** argv);
    return 0;
}