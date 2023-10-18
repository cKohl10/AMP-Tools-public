#pragma once

#include "AMPCore.h"
#include "ShareCore.h"
#include <vector>
#include <unordered_map>

struct HashNode{
    HashNode();
    HashNode(std::pair<int, int> key, int heuristic);
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

        //clear hash table
        void clearHashTable();

    private:
        // Create an unordered map with pairs of integers as keys and vectors of integers as values
        std::map<std::pair<int, int>, int> hashTable;
};