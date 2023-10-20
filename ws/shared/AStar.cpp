#include "Astar.h"

AstarNode::AstarNode(amp::Node node, double edgeWeightFromStart, double heuristic, double edgeWeightFromParent, AstarNode* parent, std::vector<AstarNode*> children)
        : node(node), edgeWeightFromStart(edgeWeightFromStart), heuristic(heuristic), edgeWeightFromParent(edgeWeightFromParent), parent(parent), children(children) {}

// MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
//     return GraphSearchResult();
// }

MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
    
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

        //std::cout << "Processing Node " << parent->node << std::endl;

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

void MyAStarAlgo::printPath(AstarNode* node){
    std::cout << "Total Cost of " << node->edgeWeightFromStart << std::endl;
    std::cout << "Path: " << node->node;
    while (node->parent != nullptr){
        node = node->parent;
        std::cout  << " -> " << node->node;
    }
    std::cout << std::endl;
}
