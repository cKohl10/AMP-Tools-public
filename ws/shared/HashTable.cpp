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
