#include "HashTable.h"

HashTable2D::HashTable2D() {
    
}

HashNode::HashNode(std::pair<int, int> key, int heuristic) {
    this->key = key;
    this->heuristic = heuristic;
}

HashNode::HashNode() {
    this->key = std::make_pair(-1, -1);
    this->heuristic = -1;
}

void HashNode::print() {
    std::cout << "Key: (" << this->key.first << ", " << this->key.second << ") Value: " << this->heuristic << std::endl;
}

void HashTable2D::addToHashTable(std::pair<int,int> key, int heuristic) {
    // Check if the pair (i, j) already exists in the hash table
    auto it = hashTable.find(key);
    if (it != hashTable.end()) {
        // If the pair exists, check if the new heuristic is smaller than the current one
        if (heuristic < it->second) {
            // If the new heuristic is smaller, replace the current heuristic with the new one
            it->second = heuristic;
        }

    } else {
        // If the pair does not exist, create a new vector with k and insert it into the hash table
        hashTable.insert({key, heuristic});
    }
}

void HashTable2D::addToHashTable(HashNode hashNode) {
    // Check if the pair (i, j) already exists in the hash table
    auto it = hashTable.find(hashNode.key);
    if (it != hashTable.end()) {
        // If the pair exists, check if the new heuristic is smaller than the current one
        if (hashNode.heuristic < it->second) {
            // If the new heuristic is smaller, replace the current heuristic with the new one
            it->second = hashNode.heuristic;
        }

    } else {
        // If the pair does not exist, create a new vector with k and insert it into the hash table
        hashTable.insert({hashNode.key, hashNode.heuristic});
    }
}

int HashTable2D::getHeuristic(std::pair<int,int> key) {
    // Check if the pair (i, j) exists in the hash table
    auto it = hashTable.find(key);
    if (it != hashTable.end()) {
        // If the pair exists, return the vector
        return it->second;
    } else {
        // If the pair does not exist, return an empty vector
        return -1;
    }
}

void HashTable2D::printHashTable() {
    // Iterate through the hash table and print the keys and values
    for (auto it = hashTable.begin(); it != hashTable.end(); it++) {
        std::cout << "Key: (" << it->first.first << ", " << it->first.second << ") Value: " << it->second << std::endl;
    }
}

void HashTable2D::clearHashTable() {
    hashTable.clear();
}

bool HashTable2D::checkIfKeyExists(std::pair<int,int> key) {
    // Check if the pair (i, j) exists in the hash table
    auto it = hashTable.find(key);
    if (it != hashTable.end()) {
        // If the pair exists, return true
        return true;
    } else {
        // If the pair does not exist, return false
        return false;
    }
}

void HashTable2D::propogateHash(const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, std::vector<std::pair<int, int>> allNeighborOrder) {
    //############## Build the Grid Space ###############
    // std::cout << "Making a plan in C-Space..." << std::endl;
    // std::cout << "Cspace Bounds: x0->[" << grid_cspace.x0Bounds().first << ", " << grid_cspace.x0Bounds().second << "] x1->[" << grid_cspace.x1Bounds().first << ", " << grid_cspace.x1Bounds().second << "]" << std::endl;
    // std::cout << "Cspace Size: (x0_cells, x1_cells)->(" << grid_cspace.size().first << ", " << grid_cspace.size().second << ")" << std::endl;

    //Make a queue of nodes to visit
    std::vector<HashNode> nodeQueue;

    //Convert start and stop nodes to cell indices
    HashNode goalNode(grid_cspace.getCellFromPoint(q_goal[0], q_goal[1]), 2);

    //Create a hashtable to keep track of visited nodes
    std::cout << "Creating hashtable..." << std::endl;
    nodeQueue.push_back(goalNode);


    //Breadth first completion of cells
    HashNode currentNode;
    while(nodeQueue.size() > 0){

        //Get the current node and pop from queue
        currentNode = nodeQueue[0];
        nodeQueue.erase(nodeQueue.begin());

        // //check if key exisits
        if (getHeuristic(currentNode.key) <= currentNode.heuristic && getHeuristic(currentNode.key) != -1){
            //If the node is in collision, remove it from the queue and continue
            //std::cout << "key  (" << currentNode.key.first << ", " << currentNode.key.second << ") already exists!" << std::endl;
            continue;
        }

        
        //Check if the current node is outside of the boundries
        if (currentNode.key.first >= grid_cspace.size().first || currentNode.key.first < 0 || currentNode.key.second >= grid_cspace.size().second || currentNode.key.second < 0){
            //std::cout << "key  (" << currentNode.key.first << ", " << currentNode.key.second << ") is out of bounds!" << std::endl;
            //If the node is outside of the boundries, remove it from the queue and continue
            continue;
        }

        //Check if the current node is in collision
        if (grid_cspace.operator()(currentNode.key.first, currentNode.key.second)){
            //If the node is in collision, remove it from the queue and continue
            //std::cout << "key  (" << currentNode.key.first << ", " << currentNode.key.second << ") is in collision!" << std::endl;
            currentNode.heuristic = 1;
            addToHashTable(currentNode);
            continue;
        }

        //Add the node to the hashtable
        addToHashTable(currentNode);

        //Add all neighbors to the queue
        for (int i = 0; i < allNeighborOrder.size(); i++){
            //Get the neighbor index
            std::pair<int, int> neighborIndex = std::make_pair(currentNode.key.first + allNeighborOrder[i].first, currentNode.key.second + allNeighborOrder[i].second);

            //Check if the neighbor is in the hashtable
            // if (hash.getHeuristic(neighborIndex) != -1){
            //     //If the neighbor is in the hashtable, continue
            //     continue;
            // }

            //Add the neighbor to the queue
            nodeQueue.push_back(HashNode(neighborIndex, currentNode.heuristic+1));
        }

    }
}

void HashTable2D::propogateHashTorus(const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, std::vector<std::pair<int, int>> allNeighborOrder) {
    //############## Build the Grid Space ###############
    // std::cout << "Making a plan in C-Space..." << std::endl;
    // std::cout << "Cspace Bounds: x0->[" << grid_cspace.x0Bounds().first << ", " << grid_cspace.x0Bounds().second << "] x1->[" << grid_cspace.x1Bounds().first << ", " << grid_cspace.x1Bounds().second << "]" << std::endl;
    // std::cout << "Cspace Size: (x0_cells, x1_cells)->(" << grid_cspace.size().first << ", " << grid_cspace.size().second << ")" << std::endl;

    //Make a queue of nodes to visit
    std::vector<HashNode> nodeQueue;

    //Convert start and stop nodes to cell indices
    HashNode goalNode(grid_cspace.getCellFromPoint(q_goal[0], q_goal[1]), 2);

    //Create a hashtable to keep track of visited nodes
    std::cout << "Creating hashtable..." << std::endl;
    nodeQueue.push_back(goalNode);


    //Breadth first completion of cells
    HashNode currentNode;
    while(nodeQueue.size() > 0){

        //Get the current node and pop from queue
        currentNode = nodeQueue[0];
        nodeQueue.erase(nodeQueue.begin());

        //Take care of wrapping bounds
        currentNode.key = std::make_pair((currentNode.key.first+grid_cspace.size().first) % grid_cspace.size().first, (currentNode.key.second+grid_cspace.size().second) % grid_cspace.size().second);

        // //check if key exisits
        if (getHeuristic(currentNode.key) <= currentNode.heuristic && getHeuristic(currentNode.key) != -1){
            //If the node is in collision, remove it from the queue and continue
            //std::cout << "key  (" << currentNode.key.first << ", " << currentNode.key.second << ") already exists!" << std::endl;
            continue;
        }

        
        //Check if the current node is outside of the boundries
        // if (currentNode.key.first >= grid_cspace.size().first || currentNode.key.first < 0 || currentNode.key.second >= grid_cspace.size().second || currentNode.key.second < 0){
        //     //std::cout << "key  (" << currentNode.key.first << ", " << currentNode.key.second << ") is out of bounds!" << std::endl;
        //     //If the node is outside of the boundries, remove it from the queue and continue
        //     continue;
        // }

        //Check if the current node is in collision
        if (grid_cspace.operator()(currentNode.key.first, currentNode.key.second)){
            //If the node is in collision, remove it from the queue and continue
            //std::cout << "key  (" << currentNode.key.first << ", " << currentNode.key.second << ") is in collision!" << std::endl;
            currentNode.heuristic = 1;
            addToHashTable(currentNode);
            continue;
        }

        //Add the node to the hashtable
        addToHashTable(currentNode);

        //Add all neighbors to the queue
        for (int i = 0; i < allNeighborOrder.size(); i++){
            //Get the neighbor index
            std::pair<int, int> neighborIndex = std::make_pair(currentNode.key.first + allNeighborOrder[i].first, currentNode.key.second + allNeighborOrder[i].second);

            //Check if the neighbor is in the hashtable
            // if (hash.getHeuristic(neighborIndex) != -1){
            //     //If the neighbor is in the hashtable, continue
            //     continue;
            // }

            //Add the neighbor to the queue
            nodeQueue.push_back(HashNode(neighborIndex, currentNode.heuristic+1));
        }

    }
}

HashNode HashTable2D::traverseHash(HashNode q, std::vector<std::pair<int, int>> allNeighborOrder){

    //print current node
    //q.print();

    //Get the neighbors of the node
    std::vector<HashNode> neighbors;
    for (int i = 0; i < allNeighborOrder.size(); i++){
        //Get the neighbor index
        std::pair<int, int> neighborIndex = std::make_pair(q.key.first + allNeighborOrder[i].first, q.key.second + allNeighborOrder[i].second);

        //Check if the neighbor is in the hashtable or a collision
        if (getHeuristic(neighborIndex) == -1 || getHeuristic(neighborIndex) == 1){
            //If the neighbor is in the hashtable, continue
            continue;
        }

        //Add the neighbor to the queue
        neighbors.push_back(HashNode(neighborIndex, getHeuristic(neighborIndex)));
    }

    //Find the neighbor with the smallest heuristic
    HashNode smallestNeighbor;
    for (int i = 0; i < neighbors.size(); i++){
        //print neighbor node:
        //std::cout << "Neighbor " << i << ": ";
        //neighbors[i].print();

        //Check if the neighbor is the smallest distance
        if (neighbors[i].heuristic <= smallestNeighbor.heuristic || smallestNeighbor.heuristic == -1){
            smallestNeighbor = neighbors[i];
        }
    }

    if (smallestNeighbor.heuristic == -1){
        std::cout << "No neighbors found!" << std::endl;
        return smallestNeighbor;
    }

    // std::cout << "Smallest Neighbor: ";
    // smallestNeighbor.print();
    // std::cout << std::endl;

    //Return the smallest neighbor
    return smallestNeighbor;
}

HashNode HashTable2D::traverseHashTorus(HashNode q, std::vector<std::pair<int, int>> allNeighborOrder, const amp::GridCSpace2D& grid_cspace){

    //print current node
    //q.print();

    //Get the neighbors of the node
    std::vector<HashNode> neighbors;
    for (int i = 0; i < allNeighborOrder.size(); i++){
        //Get the neighbor index
        std::pair<int, int> neighborIndex = std::make_pair((q.key.first + allNeighborOrder[i].first + grid_cspace.size().first)%grid_cspace.size().first, (q.key.second + allNeighborOrder[i].second+grid_cspace.size().second)%grid_cspace.size().second);

        //Check if the neighbor is in the hashtable or a collision
        if (getHeuristic(neighborIndex) == -1 || getHeuristic(neighborIndex) == 1){
            //If the neighbor is in the hashtable, continue
            continue;
        }

        //Add the neighbor to the queue
        neighbors.push_back(HashNode(neighborIndex, getHeuristic(neighborIndex)));
    }

    //Find the neighbor with the smallest heuristic
    HashNode smallestNeighbor;
    for (int i = 0; i < neighbors.size(); i++){
        //print neighbor node:
        //std::cout << "Neighbor " << i << ": ";
        //neighbors[i].print();

        //Check if the neighbor is the smallest distance
        if (neighbors[i].heuristic <= smallestNeighbor.heuristic || smallestNeighbor.heuristic == -1){
            smallestNeighbor = neighbors[i];
        }
    }

    if (smallestNeighbor.heuristic == -1){
        std::cout << "No neighbors found!" << std::endl;
        return smallestNeighbor;
    }

    // std::cout << "Smallest Neighbor: ";
    // smallestNeighbor.print();
    // std::cout << std::endl;

    //Return the smallest neighbor
    return smallestNeighbor;
}

Eigen::Vector2d HashTable2D::getPosFromKey(std::pair<int, int> key, const amp::GridCSpace2D& grid_cspace) {
    //Get the bounds of the cspace
    std::pair<double, double> x0_bounds = grid_cspace.x0Bounds();
    std::pair<double, double> x1_bounds = grid_cspace.x1Bounds();

    //Get the size of the cspace
    std::pair<std::size_t, std::size_t> cspace_size = grid_cspace.size();

    //Calculate the step size for each dimension
    double x0_step = (x0_bounds.second - x0_bounds.first)/cspace_size.first;
    double x1_step = (x1_bounds.second - x1_bounds.first)/cspace_size.second;

    //Calculate the position from the key
    Eigen::Vector2d pos;
    pos[0] = x0_bounds.first + key.first*x0_step;
    pos[1] = x1_bounds.first + key.second*x1_step;

    return pos;
}
