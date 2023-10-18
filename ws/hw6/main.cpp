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
            std::cout << std::endl << "New getCellFromPoint() function called!" << std::endl;
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
            hash.propogateHash(q_goal, grid_cspace, allNeighborOrder);

            //Print the hashtable
            //hash.printHashTable();

            //################# Build the path #################
            amp::Path2D path;

            //std::vector<double> startingState = 
            //std::pair<double, double> startIndex = std::make_pair(q_init[0], q_init[1]);
            std::pair<int, int> startIndex = grid_cspace.getCellFromPoint(q_init[0], q_init[1]);

            //print the initial position and startIndex
            std::cout << "Initial position: (" << q_init[0] << ", " << q_init[1] << ")" << std::endl;
            std::cout << "Start index: (" << startIndex.first << ", " << startIndex.second << ")" << std::endl;

            //Start the path descent from the start node
            path.waypoints.push_back(q_init);

            HashNode q(startIndex, hash.getHeuristic(startIndex));

            while (q.heuristic > 2){
                //Get the next node
                q = hash.traverseHash(q, allNeighborOrder);

                //print current angles
                std::cout << "Angles-> (" << q.key.first << ", " << q.key.second << ")" << std::endl << std::endl;

                //Add the node to the path
                path.waypoints.push_back(hash.getPosFromKey(q.key, grid_cspace));
            }

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

    //Vector of bools to make it easier to run the right graphs
    std::vector<bool> run = {false, false, false};

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
    if (run[2]){
        std::cout << "Starting Problem 2: " << std::endl;

        //Create the environment
        amp::Problem2D env3a = HW6::getHW4Problem1();
        //std::vector<Eigen::Vector2d> v = {Eigen::Vector2d(1.0, 1.0), Eigen::Vector2d(-1.0, 1.0), Eigen::Vector2d(-1.0, -1.0), Eigen::Vector2d(1.0, -1.0)};
        //env3a.obstacles.push_back(amp::Obstacle2D(v));
        amp::Environment2D env3b = amp::HW4::getEx3Workspace2();
        amp::Environment2D env3c = amp::HW4::getEx3Workspace3();

        //Create the link manipulator
        std::vector<double> linkLengths3a = {1.0, 1.0};
        LinkMan manipulator3a = LinkMan(linkLengths3a);
        CSpaceConstructor constructor = CSpaceConstructor();

        //Run WaveFront
        MyManipWFAlgo manipWF = MyManipWFAlgo();

        // Make a path
        amp::Path2D path3a = manipWF.planInCSpace(env3a.q_init, env3a.q_goal, *constructor.construct(manipulator3a, env3a));
        Visualizer::makeFigure(env3a, path3a);



        amp::Visualizer::makeFigure(*constructor.construct(manipulator3a, env3a));
        //amp::Visualizer::makeFigure(*constructor.construct(manipulator3a, env3b));
        //amp::Visualizer::makeFigure(*constructor.construct(manipulator3a, env3c));
    }

    //Show figures
    Visualizer::showFigures();

    //amp::HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("carson.kohlbrenner@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple("hey therre"), std::make_tuple());
    return 0;
}