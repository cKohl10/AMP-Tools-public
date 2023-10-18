#include "AMPCore.h"
#include "ShareCore.h"
#include "hw/HW6.h"
#include "hw/HW4.h"
#include "hw/HW2.h"

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
            std::cout << "x0: " << x0 << " x1: " << x1 << std::endl;
            std::cout << "x0_step: " << x0_step << " x1_step: " << x1_step << std::endl;
            std::cout << "m_x0_bounds.first: " << m_x0_bounds.first << " m_x0_bounds.second: " << m_x0_bounds.second << std::endl;
            std::cout << "m_x1_bounds.first: " << m_x1_bounds.first << " m_x1_bounds.second: " << m_x1_bounds.second << std::endl;
            std::cout << "i: " << i << " j: " << j << std::endl << std::endl;

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
            std::cout << "x0_cells: " << x0_cells << " x1_cells: " << x1_cells << std::endl;
            std::cout << "x0_min: " << x0_min << " x0_max: " << x0_max << std::endl;
            std::cout << "x1_min: " << x1_min << " x1_max: " << x1_max << std::endl << std::endl;

            std::unique_ptr<amp::GridCSpace2D> Gptr = std::make_unique<MyGridCSpace>(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max);

            //Check that Gptr has the same values
            std::cout << "Gptr->size().first: " << Gptr->size().first << " Gptr->size().second: " << Gptr->size().second << std::endl;

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
        virtual amp::Path2D plan(const amp::Problem2D& problem) override {
            return amp::Path2D();
        }

        //Recursive function that will traverse all cells and marked them with a heuristic
        // void checkCell(std::pair<int, int> index, std::vector<std::pair<int, int>> neighborOrder, const amp::GridCSpace2D& grid_cspace, std::pair<int, int> goalNode, int distFromStart){
           
        //     //Check if the cell is the goal node
        //     if (index == goalNode){
        //         return;
        //     }

        //     //Check if the cell is in collision
        //     if (grid_cspace.operator()(index.first, index.second)){
        //         return;
        //     }

        //     hash.addToHashTable(index, distanceFromStart);

        //     //Check all neighbors
        //     for (int i = 0; i < neighborOrder.size(); i++){
        //         //Get the neighbor index
        //         std::pair<int, int> neighborIndex = std::make_pair(index.first + neighborOrder[i].first, index.second + neighborOrder[i].second);

        //         //Call the recursive function on the neighbor
        //         checkCell(neighborIndex, neighborOrder, grid_cspace, goalNode, distFromStart+1);

        //     }
        // }

        // You need to implement here
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override {
            amp::Path2D path;

            std::cout << "Making a plan in C-Space..." << std::endl;

            //Build a graph starting from q_init keeping track of the distance traveled
            //CellGraph graph(grid_cspace.getCellFromPoint(q_init[0], q_init[1]), grid_cspace.getCellFromPoint(q_goal[0], q_goal[1]), grid_cspace);

            //Make a queue of nodes to visit
            std::vector<HashNode> nodeQueue;

            //Convert start and stop nodes to cell indices
            HashNode startNode(grid_cspace.getCellFromPoint(q_init[0], q_init[1]), 0);
            HashNode goalNode(grid_cspace.getCellFromPoint(q_goal[0], q_goal[1]), INT16_MAX);
            nodeQueue.push_back(startNode);

            //Get the edges of the workspace


            //Make the array of neighbors to visit
            std::vector<std::pair<int, int>> edgeNeighborOrder;
            std::vector<std::pair<int, int>> allNeighborOrder;

            //Initialize the neighbor order
            allNeighborOrder.push_back(std::make_pair(0, 1));
            allNeighborOrder.push_back(std::make_pair(1, 1));
            allNeighborOrder.push_back(std::make_pair(1, 0));
            allNeighborOrder.push_back(std::make_pair(1, -1));
            allNeighborOrder.push_back(std::make_pair(0, -1));
            allNeighborOrder.push_back(std::make_pair(-1, -1));
            allNeighborOrder.push_back(std::make_pair(-1, 0));
            allNeighborOrder.push_back(std::make_pair(-1, 1));

            //Initialize the edge neighbor order
            edgeNeighborOrder.push_back(std::make_pair(0, 1));
            edgeNeighborOrder.push_back(std::make_pair(1, 0));
            edgeNeighborOrder.push_back(std::make_pair(0, -1));
            edgeNeighborOrder.push_back(std::make_pair(-1, 0));

            //Create a hashtable to keep track of visited nodes
            std::cout << "Creating hashtable..." << std::endl;
            // std::unordered_map<std::pair<int, int>, int> hashy;
            hash.clearHashTable();
            hash.addToHashTable(startNode);


            //Breadth first completion of cells
            HashNode currentNode;
            while(nodeQueue.size() > 0){

                //Get the current node and pop from queue
                currentNode = nodeQueue[0];
                nodeQueue.erase(nodeQueue.begin());
                
                //Check if the current node is outside of the boundries
                // if (currentNode.key.first < grid_cspace. || currentNode.key.first >= grid_cspace.size().first || currentNode.key.second < 0 || currentNode.key.second >= grid_cspace.size().second){
                //     //If the node is outside of the boundries, remove it from the queue
                //     nodeQueue.erase(nodeQueue.begin());
                //     continue;
                // }
            }
            hash.printHashTable();


            
            return path;
        }

        private:
            //Create the hashtable
            HashTable2D hash;
};

class MyManipWFAlgo : public amp::ManipulatorWaveFrontAlgorithm {
    public:
        // Default ctor
        MyManipWFAlgo()
            : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyCSpaceCtor>()) {}

        // You can have custom ctor params for all of these classes
        MyManipWFAlgo(const std::string& beep) 
            : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyCSpaceCtor>()) {LOG("construcing... " << beep);}

        // This is just to get grade to work, you DO NOT need to override this method
        virtual amp::ManipulatorTrajectory2Link plan(const LinkManipulator2D& link_manipulator_agent, const amp::Problem2D& problem) override {
            return amp::ManipulatorTrajectory2Link();
        }
        
        // You need to implement here
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override {
            Path2D path;

            //Build the graph starting from q_init keeping track of the distance traveled

            
            return path;
        }

};

class MyAStarAlgo : public amp::AStar {
    public:
        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override {
            return GraphSearchResult();
        }
};

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // ############ Exercise 1 ###############
    // Create a CSpace object with grid size of 0.25 for HW2 
    {
        Problem2D problem = HW2::getWorkspace1();
        MyPointWFAlgo pointWF = MyPointWFAlgo();

        //Make a discretized space
        std::unique_ptr<GridCSpace2D> gridCSpace = pointWF.constructDiscretizedWorkspace(problem);

        // Make a path
        amp::Path2D path = pointWF.planInCSpace(problem.q_init, problem.q_goal, *gridCSpace);

        Visualizer::makeFigure(problem, path);
        Visualizer::makeFigure(*gridCSpace);


    }

    //Show figures
    Visualizer::showFigures();

    //amp::HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("nonhuman.biologic@myspace.edu", argc, argv, std::make_tuple(), std::make_tuple("hey therre"), std::make_tuple());
    return 0;
}