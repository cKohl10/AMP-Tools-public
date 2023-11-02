#include "Astar.h"
#include <queue>

AstarNode::AstarNode(amp::Node node, double edgeWeightFromStart, double heuristic, double edgeWeightFromParent, std::shared_ptr<AstarNode> parent, std::vector<std::shared_ptr<AstarNode>> children)
        : node(node), edgeWeightFromStart(edgeWeightFromStart), heuristic(heuristic), edgeWeightFromParent(edgeWeightFromParent), parent(parent), children(children) {}

// MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
//     return GraphSearchResult();
// }

void AstarNode::print(){
    std::cout << "Node: " << node << std::endl;
    std::cout << "Edge Weight From Start: " << edgeWeightFromStart << std::endl;
    std::cout << "Heuristic: " << heuristic << std::endl;
    std::cout << "Edge Weight From Parent: " << edgeWeightFromParent << std::endl;
    std::cout << "Parent: " << parent << std::endl;
    std::cout << "Children: ";
    for (int i = 0; i < children.size(); i++){
        std::cout << children[i] << " ";
    }
    std::cout << std::endl;
}

struct CompareAstarNode{
    bool operator()(std::shared_ptr<AstarNode> const& n1, std::shared_ptr<AstarNode> const& n2){
        return n1->edgeWeightFromStart + n1->heuristic > n2->edgeWeightFromStart + n2->heuristic;
    }
};

MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {

    //std::cout << "Running A* Search..." << std::endl;
    
    //Create Priority List
    //std::vector<AstarNode*> O;
    std::priority_queue<std::shared_ptr<AstarNode>, std::vector<std::shared_ptr<AstarNode>>, CompareAstarNode> O;

    //Processed List
    //std::vector<AstarNode*> C;
    std::map <amp::Node, std::shared_ptr<AstarNode>> C;

    //Start from initial Node
    std::shared_ptr<AstarNode> initNode = std::make_shared<AstarNode>(problem.init_node, 0, heuristic(problem.init_node), 0, nullptr, std::vector<std::shared_ptr<AstarNode>>());

    //Debugging
    //initNode->print();

    //Push intial node to priority list
    O.push(initNode);

    //Make a counter to determine the number of iterations to solve
    int counter = 1;

    //Repeat until prioirty list is exhausted
    while(O.size() > 0 && counter < 100000){

        //Get the current node
        std::shared_ptr<AstarNode> parent = O.top();

        //std::cout << "Processing Node " << parent->node << std::endl;

        //remove it from priority queue
        O.pop();
        parent->processed = true;
        
        //Add the parent node to the processed map
        C.insert({parent->node, parent});

        //Check if the goal node was reached
        if (parent->node == problem.goal_node){
            //std::cout << "Goal Reached!" << std::endl;
            //std::cout << "Iterations: " << counter << std::endl;
            //printPath(parent);

            //Build the path
            GraphSearchResult result;
            result.path_cost = parent->edgeWeightFromStart;
            result.success = true;
            while (parent->parent != nullptr){
                result.node_path.push_front(parent->node);
                parent = parent->parent;
            }
            result.node_path.push_front(parent->node);

            // //Delete the pointers to prevent memory leak
            // for (auto it = C.begin(); it != C.end(); it++){
            //     delete it->second;
            // }

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
                // std::cout << " ---> Child was already processed!" << std::endl;
                continue;
            }

            //Debugging
            // std::cout << "Child: " << children[i] << std::endl;
            // std::cout << "Edge: " << edges[i] << std::endl;


            //Create child node and add to parent children path
            std::shared_ptr<AstarNode> child = std::make_shared<AstarNode>(children[i], parent->edgeWeightFromStart + edges[i], heuristic(children[i]), edges[i], parent, std::vector<std::shared_ptr<AstarNode>>());
            //parent->children.push_back(child);

            // Add the child to the priority queue
            O.push(child);

        };

        //Sort the priority queue by the cost
        //std::sort(O.begin(), O.end(), [](AstarNode* a, AstarNode* b) {return a->edgeWeightFromStart + a->heuristic > b->edgeWeightFromStart + b->heuristic;});

        //Add the iteration to the counter
        counter++;
    }

    //If the priority queue is exhausted, return an empty path
    //std::cout << "No path found!" << std::endl;
    //std::cout << "Iterations: " << counter << std::endl;

    return GraphSearchResult();
}

amp::Path MyAStarAlgo::searchPath(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic, std::map<amp::Node, Eigen::VectorXd> node_map){
    
    //std::cout << "Running search before conversion..." << std::endl;
    GraphSearchResult path_searchResult = search(problem, heuristic);

    //Build the path
    //std::cout << "Beginning path conversion..." << std::endl;
    amp::Path path;

    //Loop through all the nodes
    for (amp::Node x : path_searchResult.node_path){
        //std::cout << "Iteration " << i << std::endl;
    
        //std::cout << "Adding node " << x << " to path" << std::endl;
        //Add the node to the path
        path.waypoints.push_back(node_map[x]);
    }

    return path;
}

void MyAStarAlgo::printPath(std::shared_ptr<AstarNode> node){
    std::cout << "Total Cost of " << node->edgeWeightFromStart << std::endl;
    std::cout << "Path: " << node->node;
    while (node->parent != nullptr){
        node = node->parent;
        std::cout  << " -> " << node->node;
    }
    std::cout << std::endl;
}
