#include "AMPCore.h"
#include "hw/HW6.h"
#include "hw/HW4.h"
#include "hw/HW2.h"
#include "ShareCore.h"
#include "HelpfulClass.h"

using namespace amp;

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    //Vector of bools to make it easier to run the right graphs
    std::vector<bool> run = {false, true, true};

    // ############ Exercise 1a ###############
    // Create a CSpace object with grid size of 0.25 for HW2 
    if (run[0]){
        Problem2D problem = HW2::getWorkspace1();
        MyPointWFAlgo pointWF = MyPointWFAlgo();

        //Make a discretized space
        std::unique_ptr<GridCSpace2D> gridCSpace = pointWF.constructDiscretizedWorkspace(problem);

        // Make a path
        amp::Path2D path = pointWF.planInCSpace(problem.q_init, problem.q_goal, *gridCSpace);

        Visualizer::makeFigure(problem, path);
        //Visualizer::makeFigure(*gridCSpace);

        //print path distance
        std::cout << "Path distance: " << pathDistance(path) << std::endl;
    }

        // ############ Exercise 1b ###############
    // Create a CSpace object with grid size of 0.25 for HW2 
    if (run[0]){
        Problem2D problem = HW2::getWorkspace2();
        MyPointWFAlgo pointWF = MyPointWFAlgo();

        //Make a discretized space
        std::unique_ptr<GridCSpace2D> gridCSpace = pointWF.constructDiscretizedWorkspace(problem);

        // Make a path
        amp::Path2D path = pointWF.planInCSpace(problem.q_init, problem.q_goal, *gridCSpace);

        Visualizer::makeFigure(problem, path);
        //Visualizer::makeFigure(*gridCSpace);

        //print path distance
        std::cout << "Path distance: " << pathDistance(path) << std::endl;
    }

    //################# Exercise 2 #################//
    //Create the link manipulator
    std::vector<double> linkLengths = {1.0, 1.0};
    LinkMan manipulator = LinkMan(linkLengths);
    CSpaceConstructor constructor = CSpaceConstructor();
    //Run WaveFront
    MyManipWFAlgo manipWF = MyManipWFAlgo();

    if (run[1]){
        std::cout << "Starting Problem 2a: " << std::endl;

        //Create the environment
        amp::Problem2D problem = HW6::getHW4Problem1();

        // Get the initial state from IK
        amp::ManipulatorState init_state = manipulator.getConfigurationFromIK(problem.q_init);

            // Get the goal state from IK
        amp::ManipulatorState goal_state = manipulator.getConfigurationFromIK(problem.q_goal);

        // Make a path
        amp::Path2D path = manipWF.planInCSpace(convert(init_state), convert(goal_state), *constructor.construct(manipulator, problem));
        HW6::checkLinkManipulatorPlan(path, manipulator, problem);
        //Visualizer::makeFigure(problem, path);

        amp::Visualizer::makeFigure(*constructor.construct(manipulator, problem), path);
        amp::Visualizer::makeFigure(problem, manipulator, path);
    }

    if (run[1]){
        std::cout << "Starting Problem 2b: " << std::endl;

        //Create the environment
        amp::Problem2D problem = HW6::getHW4Problem2();

        // Get the initial state from IK
        amp::ManipulatorState init_state = manipulator.getConfigurationFromIK(problem.q_init);

            // Get the goal state from IK
        amp::ManipulatorState goal_state = manipulator.getConfigurationFromIK(problem.q_goal);

        // Make a path
        amp::Path2D path = manipWF.planInCSpace(convert(init_state), convert(goal_state), *constructor.construct(manipulator, problem));
        HW6::checkLinkManipulatorPlan(path, manipulator, problem);
        //Visualizer::makeFigure(problem, path);

        amp::Visualizer::makeFigure(*constructor.construct(manipulator, problem), path);
        amp::Visualizer::makeFigure(problem, manipulator, path);
    }

    if (run[1]){
        std::cout << std::endl << "Starting Problem 2c: " << std::endl;

        //Create the environment
        amp::Problem2D problem = HW6::getHW4Problem3();

        // Get the initial state from IK
        amp::ManipulatorState init_state = manipulator.getConfigurationFromIK(problem.q_init);

            // Get the goal state from IK
        amp::ManipulatorState goal_state = manipulator.getConfigurationFromIK(problem.q_goal);

        // Make a path
        amp::Path2D path = manipWF.planInCSpace(convert(init_state), convert(goal_state), *constructor.construct(manipulator, problem));
        HW6::checkLinkManipulatorPlan(path, manipulator, problem);
        //Visualizer::makeFigure(problem, path);

        amp::Visualizer::makeFigure(*constructor.construct(manipulator, problem), path);
        amp::Visualizer::makeFigure(problem, manipulator, path);
    }

    if (run[2]){
        std::cout << "Starting Problem 3: " << std::endl;

        //Create the environment
        amp::ShortestPathProblem problem = HW6::getEx3SPP();

        //Get the search heuristics
        amp::LookupSearchHeuristic heuristic = HW6::getEx3Heuristic();
        
        //Print the heuristic in all the nodes
        // for (int i = 0; i < 12; i++){
        //     std::cout << "Heuristic at node " << i << ": " << heuristic.operator()(i) << std::endl;
        // }

        //problem.graph->print();

        MyAStarAlgo aStar = MyAStarAlgo();

        //Starting the A* search
        std::cout << std::endl << std::endl << "Searching Graph with A*..." << std::endl;
        amp::AStar::GraphSearchResult resultA = aStar.search(problem, heuristic);

        //Check the result path
        std::cout << "Returned Path: ";
        for (int i : resultA.node_path){
            std::cout << i << " -> ";
        }
        std::cout << std::endl;



        //Starting the Djikstra's Search
        std::cout << std::endl << std::endl << "Searching Graph with Djikstra's..." << std::endl;
        
        //set all heuristic values to zero
        std::vector<amp::Node> nodes = problem.graph->nodes();
        for (int i = 0; i < nodes.size(); i++){
            heuristic.heuristic_values[nodes[i]] = 0;
        }

        //Searching the path
        amp::AStar::GraphSearchResult resultD = aStar.search(problem, heuristic);

        //Check the result path
        std::cout << "Returned Path: ";
        for (int i : resultD.node_path){
            std::cout << i << " -> ";
        }
        std::cout << std::endl;

    }

    //Show figures
    Visualizer::showFigures();

    //amp::HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("carson.kohlbrenner@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple("hey therre"), std::make_tuple());
    return 0;
}