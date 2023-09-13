#include "PolyGraph.h"

//Contructor
PolyGraph::PolyGraph(amp::Problem2D problem){
    for (int i = 0; i < problem.obstacles.size(); i++){
        obstacles.push_back(problem.obstacles[i]);
        visited.push_back(false);
    }
}

//Depth First Search
std::vector<amp::Obstacle2D> PolyGraph::DFS(int i, std::vector<amp::Obstacle2D> group){
    group.push_back(obstacles[i]);

    for (int j = 0; j < obstacles.size(); j++){
        if (visited[j] == false){
            if (polygonOverlap(obstacles[i], obstacles[j])){
                visited[j] = true;
                group = DFS(j, group);
            }
        }
    }

    return group;
}

//Sorts the obstacles into groups of overlapping polygons
void PolyGraph::sort(){
    int counter = 0;

    for (int i = 0; i < obstacles.size(); i++){
        if (visited[i] == false){
            visited[i] = true;
            std::vector<amp::Obstacle2D> group;
            obstacleGroups.push_back(DFS(i, group));
            counter++;
        }
    }

    return;

}

//Finds all vertices and intersection points on the overlapping polygon sets and adds them to a set of vertices
void PolyGraph::findVertices(){
    for (int i = 0; i < obstacleGroups.size(); i++){
        std::vector<Eigen::Vector2d> vertexSet;
        for (int j = 0; j < obstacleGroups[i].size(); j++){
            for (int k = 0; k < obstacleGroups[i][j].verticesCCW().size(); k++){
                vertexSet.push_back(obstacleGroups[i][j].verticesCCW()[k]);
            }
        }
        vertices.push_back(vertexSet);
    }
}

//Find the intersection points of all polygons in the graph
void PolyGraph::findIntersectionVertices(){
    
}

//Create a common contour of all the vertices in a set and remove ones that have not been used
void PolyGraph::commonContour(){
    std::vector<std::vector<bool>> vertexVisited;

    // Set all the vertices to false in the vertexVisited vector
    for (int i = 0; i < vertices.size(); i++){
        std::vector<bool> visited;
        for (int j = 0; j < vertices[i].size(); j++){
            visited.push_back(false);
        }
        vertexVisited.push_back(visited);
    }

    // Find the vertex in the vertices vector that has the smallest distance from the origin point and use that as the bottom left point
    int minIndex = 0;
    int lastMinIndex = 0;
    double minDistance;
    std::vector<int> contourStartDex; //Starting point of the new contour
    std::vector<int> contourStart2Dex; //Next line point of the new contour

    for (int i = 0; i < vertices.size(); i++){
        for (int j = 0; j < vertices[i].size(); j++){
            if (j == 0){
                minDistance = vertices[i][j].norm();
            }
            else{
                if (vertices[i][j].norm() < minDistance){
                    minDistance = vertices[i][j].norm();
                    lastMinIndex = minIndex;
                    minIndex = j;
                }
            }
        }
        contourStartDex.push_back(minIndex);
        contourStart2Dex.push_back(lastMinIndex);
    }

    //Starting from the contour start point, find the angle of every unvisited vertice 
    //and push the one with the lowest internal angle to the contour vector
    std::vector<Eigen::Vector2d> newObstacleVertices;

    //Loop through each object set
    for (int i = 0; i < vertices.size(); i++){

        //Loop through each vertex in the object set starting from the contour start point
        int index = contourStartDex[i];
        int lastDex = contourStart2Dex[i];
        bool complete = false;

        //Make a parameterized line from the starting vertice to the next starting vertice
        using Line2 = Eigen::ParametrizedLine<double, 2>;
        Line2 lineLast(vertices[i][index], vertices[i][contourStart2Dex[i]]);

        int iter = 0;
        //Loop through all unseen vertices and find the internal angle between them
        while(complete == false){
            //Make a parameterized line 
            Line2 lineNew(vertices[i][index], vertices[i][index+1]);
        }
    }
}