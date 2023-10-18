#pragma once

#include "AMPCore.h"
#include "ShareCore.h"
#include <vector>
#include <unordered_map>

struct HashNode{
    HashNode();
    HashNode(std::pair<int, int> key, int heuristic);
    void print();

    std::pair<int, int> key;
    int heuristic;
};


class HashTable2D{
    public: 
        HashTable2D();
        void addToHashTable(std::pair<int,int> key, int heuristic);
        void addToHashTable(HashNode hashNode);
        int getHeuristic(std::pair<int,int> key);
        void printHashTable();
        bool checkIfKeyExists(std::pair<int,int> key);

        void propogateHash(const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, std::vector<std::pair<int, int>> allNeighborOrder);

        Eigen::Vector2d getPosFromKey(std::pair<int, int> key, const amp::GridCSpace2D& grid_cspace);
        HashNode traverseHash(HashNode q, std::vector<std::pair<int, int>> allNeighborOrder);


        //clear hash table
        void clearHashTable();

    private:
        // Create an unordered map with pairs of integers as keys and vectors of integers as values
        std::map<std::pair<int, int>, int> hashTable;
};