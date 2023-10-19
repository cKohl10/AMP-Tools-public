#include "AMPCore.h"
#include "hw/HW6.h"
#include "hw/HW4.h"
#include "hw/HW2.h"
#include "ShareCore.h"
#include "HelpfulClass.h"

using namespace amp;

class MyGridCSpace : public amp::GridCSpace2D {
    public:
        //MyGridCSpace()
        //    : amp::GridCSpace2D(1, 1, 0.0, 1.0, 0.0, 1.0) {}
        MyGridCSpace(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
            : amp::GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max) {}

        //Finds the cell that the point given is in
        virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const {

            //Calculate the step size for each dimension
            double x0_step = (m_x0_bounds.second - m_x0_bounds.first)/size().first;
            double x1_step = (m_x1_bounds.second - m_x1_bounds.first)/size().second;

            //Calculate the cell indices and floor them
            std::size_t i = (x0 - m_x0_bounds.first)/x0_step;
            std::size_t j = (x1 - m_x1_bounds.first)/x1_step;

            //Debuging:
            //std::cout << std::endl << "New getCellFromPoint() function called!" << std::endl;
            //std::cout << "x0: " << x0 << " x1: " << x1 << std::endl;
            //std::cout << "x0_step: " << x0_step << " x1_step: " << x1_step << std::endl;
            //std::cout << "m_x0_bounds.first: " << m_x0_bounds.first << " m_x0_bounds.second: " << m_x0_bounds.second << std::endl;
            //std::cout << "m_x1_bounds.first: " << m_x1_bounds.first << " m_x1_bounds.second: " << m_x1_bounds.second << std::endl;
            //std::cout << "i: " << i << " j: " << j << std::endl << std::endl;

            return std::make_pair(i, j);
        }
};

class MyCSpaceCtor : public amp::GridCSpace2DConstructor {
    public:
        virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) override {

            return std::make_unique<MyGridCSpace>(1, 1, 0.0, 1.0, 0.0, 1.0);
        }
};

class MyPointWFAlgo : public amp::PointWaveFrontAlgorithm {
    public:
        // Default ctor
        MyPointWFAlgo() {}

        virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment) override {
            
            //Create the discretized parameters for translating robot
            double stepSize = 0.25;
            double cornerSize = stepSize/2;
            double x0_min = environment.x_min; 
            double x0_max = environment.x_max; 
            double x1_min = environment.y_min; 
            double x1_max = environment.y_max; 
            std::size_t x0_cells = ceil((x0_max - x0_min)/stepSize); 
            std::size_t x1_cells = ceil((x1_max - x1_min)/stepSize); 

            //Debuging:
            //std::cout << "x0_cells: " << x0_cells << " x1_cells: " << x1_cells << std::endl;
            //std::cout << "x0_min: " << x0_min << " x0_max: " << x0_max << std::endl;
            //std::cout << "x1_min: " << x1_min << " x1_max: " << x1_max << std::endl << std::endl;

            std::unique_ptr<amp::GridCSpace2D> Gptr = std::make_unique<MyGridCSpace>(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max);

            //Check that Gptr has the same values
            //std::cout << "Gptr->size().first: " << Gptr->size().first << " Gptr->size().second: " << Gptr->size().second << std::endl;

            //Check for collisions
            for (int i = 0; i < int(x0_cells); i++){
                for (int j = 0; j < int(x1_cells); j++){

                    //Calculate the configuration
                    double x0 = x0_min + (i)*stepSize;
                    double x1 = x1_min + (j)*stepSize;
                    Eigen::Vector2d pos = Eigen::Vector2d(x0, x1);
                    
                    //Output pos values
                    //std::cout << "pos.first: " << pos.first << " pos.second: " << pos.second << std::endl;
                    


                    if (collisionDetectedPoint(environment.obstacles, pos) || collisionDetectedPoint(environment.obstacles, pos+Eigen::Vector2d(-cornerSize,-cornerSize)) || collisionDetectedPoint(environment.obstacles, pos+Eigen::Vector2d(-cornerSize,cornerSize)) || collisionDetectedPoint(environment.obstacles, pos+Eigen::Vector2d(cornerSize,-cornerSize)) || collisionDetectedPoint(environment.obstacles, pos+Eigen::Vector2d(cornerSize,cornerSize)) ){
                        Gptr->operator()(i,j) = true;
                    }
                }
            }

            return Gptr;
        }

        // This is just to get grade to work, you DO NOT need to override this method
        // virtual amp::Path2D plan(const amp::Problem2D& problem) override {
        //     std::cout << "Point Wave Front being graded with overided" << std::endl;
        //     return amp::Path2D();
        // }

        // You need to implement here
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override {

            //############## Build the Grid Space as a Hash Table ###############
            HashTable2D hash;

            //Make the array of neighbors to visit
            std::vector<std::pair<int, int>> allNeighborOrder;
            bool edgeNeighbor = true;

            //Initialize the edge neighbor order
            if(edgeNeighbor){
                allNeighborOrder.push_back(std::make_pair(0, 1));
                allNeighborOrder.push_back(std::make_pair(1, 0));
                allNeighborOrder.push_back(std::make_pair(0, -1));
                allNeighborOrder.push_back(std::make_pair(-1, 0));
            } else{
                //Initialize the neighbor order
                allNeighborOrder.push_back(std::make_pair(0, 1));
                allNeighborOrder.push_back(std::make_pair(1, 1));
                allNeighborOrder.push_back(std::make_pair(1, 0));
                allNeighborOrder.push_back(std::make_pair(1, -1));
                allNeighborOrder.push_back(std::make_pair(0, -1));
                allNeighborOrder.push_back(std::make_pair(-1, -1));
                allNeighborOrder.push_back(std::make_pair(-1, 0));
                allNeighborOrder.push_back(std::make_pair(-1, 1));
            }

            //Propogate the hash table
            hash.propogateHash(q_goal, grid_cspace, allNeighborOrder);

            //Print the hashtable
            //hash.printHashTable();

            //################# Build the path #################
            amp::Path2D path;

            std::pair<int, int> startIndex = grid_cspace.getCellFromPoint(q_init[0], q_init[1]);

            //Start the path descent from the start node
            path.waypoints.push_back(q_init);

            HashNode q(startIndex, hash.getHeuristic(startIndex));

            while (q.heuristic > 2){
                //Get the next node
                q = hash.traverseHash(q, allNeighborOrder);

                //Add the node to the path
                path.waypoints.push_back(hash.getPosFromKey(q.key, grid_cspace));
            }

            return path;
        }
};

class MyManipWFAlgo : public amp::ManipulatorWaveFrontAlgorithm {
    public:
        // Default ctor
        MyManipWFAlgo()
            : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<CSpaceConstructor>()) {}

        // You can have custom ctor params for all of these classes
        MyManipWFAlgo(const std::string& beep) 
            : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<CSpaceConstructor>()) {LOG("construcing... " << beep);}

        // This is just to get grade to work, you DO NOT need to override this method
        // virtual amp::ManipulatorTrajectory2Link plan(const LinkManipulator2D& link_manipulator_agent, const amp::Problem2D& problem) override {
        //     return amp::ManipulatorTrajectory2Link();
        // }
        
        // You need to implement here
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override {

            //############## Build the Grid Space as a Hash Table ###############
            HashTable2D hash;

            //Make the array of neighbors to visit
            std::vector<std::pair<int, int>> allNeighborOrder;
            bool edgeNeighbor = true;

            //Initialize the edge neighbor order
            if(edgeNeighbor){
                allNeighborOrder.push_back(std::make_pair(0, 1));
                allNeighborOrder.push_back(std::make_pair(1, 0));
                allNeighborOrder.push_back(std::make_pair(0, -1));
                allNeighborOrder.push_back(std::make_pair(-1, 0));
            } else{
                //Initialize the neighbor order
                allNeighborOrder.push_back(std::make_pair(0, 1));
                allNeighborOrder.push_back(std::make_pair(1, 1));
                allNeighborOrder.push_back(std::make_pair(1, 0));
                allNeighborOrder.push_back(std::make_pair(1, -1));
                allNeighborOrder.push_back(std::make_pair(0, -1));
                allNeighborOrder.push_back(std::make_pair(-1, -1));
                allNeighborOrder.push_back(std::make_pair(-1, 0));
                allNeighborOrder.push_back(std::make_pair(-1, 1));
            }

            //Propogate the hash table
            hash.propogateHashTorus(q_goal, grid_cspace, allNeighborOrder);

            //Print the hashtable
            //hash.printHashTable();

            //################# Build the path #################
            amp::Path2D path;

            //std::vector<double> startingState = 
            //std::pair<double, double> startIndex = std::make_pair(q_init[0], q_init[1]);
            std::pair<int, int> startIndex = grid_cspace.getCellFromPoint(q_init[0], q_init[1]);
            std::pair<int, int> goalIndex = grid_cspace.getCellFromPoint(q_goal[0], q_goal[1]);

            //print the initial position and startIndex
            std::cout << "Initial position: (" << q_init[0] << ", " << q_init[1] << ")" << std::endl;
            std::cout << "Start index: (" << startIndex.first << ", " << startIndex.second << ")" << std::endl;

            //print out the goal position and index
            std::cout << "Goal position: (" << q_goal[0] << ", " << q_goal[1] << ")" << std::endl;
            std::cout << "Goal index: (" << goalIndex.first << ", " << goalIndex.second << ")" << std::endl;
            //Start the path descent from the start node
            path.waypoints.push_back(q_init);

            HashNode q(startIndex, hash.getHeuristic(startIndex));

            while (q.heuristic > 2){
                //Get the next node
                q = hash.traverseHashTorus(q, allNeighborOrder, grid_cspace);

                //print current angles
                //std::cout << "Angles-> (" << q.key.first << ", " << q.key.second << ")" << std::endl << std::endl;

                //Add the node to the path
                path.waypoints.push_back(hash.getPosFromKey(q.key, grid_cspace));
            }

            return path;
        }

};

class MyAStarAlgo : public amp::AStar {
    public:
        struct AstarNode{
            AstarNode(amp::Node node, double edgeWeightFromStart, double heuristic, double edgeWeightFromParent, AstarNode* parent, std::vector<AstarNode*> children)
                : node(node), edgeWeightFromStart(edgeWeightFromStart), heuristic(heuristic), edgeWeightFromParent(edgeWeightFromParent), parent(parent), children(children) {}
            amp::Node node;
            double edgeWeightFromStart;
            double heuristic;
            double edgeWeightFromParent;
            AstarNode* parent;
            std::vector<AstarNode*> children;
            bool processed = false;
        };

        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override {
            
            //Create Priority List
            std::vector<AstarNode*> O;

            //Processed List
            //std::vector<AstarNode*> C;
            std::map <amp::Node, AstarNode*> C;

            //Start from initial Node
            AstarNode* initNode = new AstarNode(problem.init_node, 0, heuristic(problem.init_node), 0, nullptr, std::vector<AstarNode*>());

            //Push intial node to priority list
            O.push_back(initNode);

            //Make a counter to determine the number of iterations to solve
            int counter = 1;

            //Repeat until prioirty list is exhausted
            while(O.size() > 0){

                
                //print the prioirty queue
                // std::cout << std::endl << "#### Priority Queue ##### " << std::endl;
                // for (int i = O.size()-1; i >= 0; i--){
                //     std::cout <<"   Node " << O[i]->node << ": Cost " << O[i]->edgeWeightFromStart + O[i]->heuristic << std::endl;
                // }
                // std::cout << "#########################" << std::endl <<std::endl;

                //Get the current node
                AstarNode* parent = O.back();

                std::cout << "Processing Node " << parent->node << std::endl;

                //remove it from priority queue
                O.pop_back();
                parent->processed = true;
                
                //Add the parent node to the processed map
                C.insert({parent->node, parent});

                //Check if the goal node was reached
                if (parent->node == problem.goal_node){
                    std::cout << "Goal Reached!" << std::endl;
                    std::cout << "Iterations: " << counter << std::endl;
                    printPath(parent);

                    //Build the path
                    GraphSearchResult result;
                    result.path_cost = parent->edgeWeightFromStart;
                    result.success = true;
                    while (parent->parent != nullptr){
                        result.node_path.push_front(parent->node);
                        parent = parent->parent;
                    }
                    result.node_path.push_front(parent->node);
                    return result;
                }

                //Get the children nodes
                std::vector<amp::Node> children = problem.graph->children(parent->node);
                std::vector<double> edges = problem.graph->outgoingEdges(parent->node);
                for (int i = 0; i < children.size(); i++){

                    //std::cout << "Parent Node " << parent->node << " has child " << children[i] << std::endl;

                    //Check if child node was already processed
                    auto it = C.find(children[i]);
                    if (it != C.end()) {
                        // If the node exists, check the next child
                        //std::cout << " ---> Child was already processed!" << std::endl;
                        continue;
                    }

                    //Debugging
                    //std::cout << "Child: " << children[i] << std::endl;
                    //std::cout << "Edge: " << edges[i] << std::endl;


                    //Create child node and add to parent children path
                    AstarNode* child = new AstarNode(children[i], parent->edgeWeightFromStart + edges[i], heuristic(children[i]), edges[i], parent, std::vector<AstarNode*>());
                    parent->children.push_back(child);

                    // Add the child to the priority queue
                    O.push_back(child);

                };

                //Sort the priority queue by the cost
                std::sort(O.begin(), O.end(), [](AstarNode* a, AstarNode* b) {return a->edgeWeightFromStart + a->heuristic > b->edgeWeightFromStart + b->heuristic;});

                //Add the iteration to the counter
                counter++;
            }

            //If the priority queue is exhausted, return an empty path
            std::cout << "No path found!" << std::endl;

            return GraphSearchResult();
        }

        void printPath(AstarNode* node){
            std::cout << "Total Cost of " << node->edgeWeightFromStart << std::endl;
            std::cout << "Path: " << node->node;
            while (node->parent != nullptr){
                node = node->parent;
                std::cout  << " -> " << node->node;
            }
            std::cout << std::endl;
        }
};

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    //Vector of bools to make it easier to run the right graphs
    std::vector<bool> run = {false, false, false, false};

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
    if (run[1]){
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

    if (run[2]){
        std::cout << "Starting Problem 2a: " << std::endl;

        //Create the environment
        amp::Problem2D problem = HW6::getHW4Problem1();

        // Get the initial state from IK
        amp::ManipulatorState init_state = manipulator.getConfigurationFromIK(problem.q_init);

            // Get the goal state from IK
        amp::ManipulatorState goal_state = manipulator.getConfigurationFromIK(problem.q_goal);

        // Make a path
        amp::Path2D path = manipWF.planInCSpace(convert(init_state), convert(goal_state), *constructor.construct(manipulator, problem));
        //Visualizer::makeFigure(problem, path);

        amp::Visualizer::makeFigure(*constructor.construct(manipulator, problem), path);
        amp::Visualizer::makeFigure(problem, manipulator, path);
    }

    if (run[2]){
        std::cout << "Starting Problem 2b: " << std::endl;

        //Create the environment
        amp::Problem2D problem = HW6::getHW4Problem2();

        // Get the initial state from IK
        amp::ManipulatorState init_state = manipulator.getConfigurationFromIK(problem.q_init);

            // Get the goal state from IK
        amp::ManipulatorState goal_state = manipulator.getConfigurationFromIK(problem.q_goal);

        // Make a path
        amp::Path2D path = manipWF.planInCSpace(convert(init_state), convert(goal_state), *constructor.construct(manipulator, problem));
        //Visualizer::makeFigure(problem, path);

        amp::Visualizer::makeFigure(*constructor.construct(manipulator, problem), path);
        amp::Visualizer::makeFigure(problem, manipulator, path);
    }

    if (run[2]){
        std::cout << "Starting Problem 2c: " << std::endl;

        //Create the environment
        amp::Problem2D problem = HW6::getHW4Problem3();

        // Get the initial state from IK
        amp::ManipulatorState init_state = manipulator.getConfigurationFromIK(problem.q_init);

            // Get the goal state from IK
        amp::ManipulatorState goal_state = manipulator.getConfigurationFromIK(problem.q_goal);

        // Make a path
        amp::Path2D path = manipWF.planInCSpace(convert(init_state), convert(goal_state), *constructor.construct(manipulator, problem));
        //Visualizer::makeFigure(problem, path);

        amp::Visualizer::makeFigure(*constructor.construct(manipulator, problem), path);
        amp::Visualizer::makeFigure(problem, manipulator, path);
    }

    if (run[3]){
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

    amp::HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("carson.kohlbrenner@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple("hey therre"), std::make_tuple());
    return 0;
}