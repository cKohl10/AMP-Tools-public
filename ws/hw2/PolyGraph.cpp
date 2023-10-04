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

    //Loop through all the obstacles
    for (int j = 0; j < obstacles.size(); j++){
        if (visited[j] == false){

            //Check if the obstacle overlaps with the current obstacle
            //std::cout << "Obstacle: " << i << " check collision with obstacle " << j << std::endl;
            if (polygonOverlap(obstacles[i], obstacles[j])){
                visited[j] = true;
                group.push_back(obstacles[j]);
                group = DFS(j, group);

                //print out a statemate showing the recursive chain of DFS
                std::cout << " --> " << j;
                
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
            group.push_back(obstacles[i]);
            std::cout << "Obstacle chain: " << i;

            obstacleGroups.push_back(DFS(i, group));
            counter++;

            //print out a statemate to show that the DFS is working
            std::cout << std::endl << "DFS " << i << " complete" << std::endl;
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

    //Output how many vertice sets there are
    std::cout << "Number of vertice sets: " << vertices.size() << std::endl;
    return;
}

//Find the intersection points of all polygons in the graph
void PolyGraph::findIntersectionVertices(){
    return;
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
    double lastMinDistance = INT16_MAX;
    std::vector<int> contourStartDex; //Starting point of the new contour
    std::vector<int> contourStart2Dex; //Next line point of the new contour

    //Find the starting point of the contour
    for (int i = 0; i < vertices.size(); i++){
        for (int j = 0; j < vertices[i].size(); j++){
            if (j == 0){
                minDistance = vertices[i][j].norm();
            }
            else{
                if (vertices[i][j].norm() < minDistance){
                    minDistance = vertices[i][j].norm();
                    minIndex = j;
                }
            }
        }
        contourStartDex.push_back(minIndex);
    }

    //Find the next point of the contour
    for (int i = 0; i < vertices.size(); i++){
        for (int j = 0; j < vertices[i].size(); j++){
            if ((vertices[i][j].norm() < lastMinDistance) && (lastMinDistance > minDistance)){
                lastMinDistance = vertices[i][j].norm();
                lastMinIndex = j;
            }

        }
        contourStart2Dex.push_back(lastMinDistance);
    }


    //Starting from the contour start point, find the angle of every unvisited vertice 
    //and push the one with the lowest internal angle to the contour vector
    

    //Loop through each object set
    for (int i = 0; i < vertices.size(); i++){
        //print which set is being worked on
        std::cout << "Working on set: " << i << std::endl;

        std::vector<Eigen::Vector2d> newObstacleVertices;
        //Loop through each vertex in the object set starting from the contour start point
        int index = contourStartDex[i];
        int lastDex = contourStart2Dex[i];
        newObstacleVertices.push_back(vertices[i][lastDex]);
        newObstacleVertices.push_back(vertices[i][index]);

        //Set the vertex to visited
        vertexVisited[i][index] = true;

        //Checker to see if polygon is complete
        bool complete = false;

        //Make a parameterized line from the starting vertice to the next starting vertice
        using Line2 = Eigen::ParametrizedLine<double, 2>;
        Line2 lineLast(vertices[i][lastDex], (vertices[i][index] -  vertices[i][lastDex]).normalized());


        int iter = 0;
        //Loop through all unseen vertices and find the internal angle between them
        while(complete == false){

            //Make a smallest angle variable
            double largestAngle = 0;
            int largestAngleIndex = 0;

            //Find the smallest angle between the last line and the next line for every vertex and take the largest one
            for (int j = 0; j < vertices[i].size(); j++){

                //Check if the vertex has been visited
                if (vertexVisited[i][j] == false){
                    //Make a parameterized line 
                    Line2 lineNew(vertices[i][index], (vertices[i][j] - vertices[i][index]).normalized());

                    //Find the angle between the two lines
                    double angle = 3.1415926535 - angleBetweenLines(lineLast, lineNew);

                    //print out line directions and their angle
                    std::cout << "Line " << iter << ": (" << lineLast.origin()[0] << ", " << lineLast.origin()[1] << ") --> (" << lineLast.direction()[0] << ", " << lineLast.direction()[1] << ") and (" << lineNew.origin()[0] << ", " << lineNew.origin()[1] << ") --> (" << lineNew.direction()[0] << ", " << lineNew.direction()[1] << ") with angle " << angle << std::endl;

                    //Check if the angle is the smallest angle
                    if (angle > largestAngle){
                        largestAngle = angle;
                        largestAngleIndex = j;
                    } else if (angle == largestAngle){
                        //If the angle is the same, check which one is closer to the last vertex
                        if ((vertices[i][j] - vertices[i][index]).norm() < (vertices[i][largestAngleIndex] - vertices[i][index]).norm()){
                            largestAngleIndex = j;
                        }
                    }
                }
            }

            //Add the vertex with the smallest angle to the new obstacle vertices vector
            newObstacleVertices.push_back(vertices[i][largestAngleIndex]);
            vertexVisited[i][largestAngleIndex] = true;
            lastDex = index;
            index = largestAngleIndex;

            //Make a new line with this one
            lineLast = Line2(vertices[i][lastDex], (vertices[i][index] -  vertices[i][lastDex]).normalized());

            //Exit if the last vertex is the same as the first vertex
            if (newObstacleVertices.back() == newObstacleVertices[0]){
                newObstacleVertices.pop_back();
                complete = true;
            }

            iter++;
        }

        //reverse the vector to make it counter clockwise
        std::reverse(newObstacleVertices.begin(), newObstacleVertices.end());

        //Print out the new obstacle vertices
        std::cout << "New obstacle vertices: " << std::endl;
        for (int j = 0; j < newObstacleVertices.size(); j++){
            std::cout << "(" << newObstacleVertices[j][0] << ", " << newObstacleVertices[j][1] << ")" << std::endl;
        }

        //Add the new obstacle vertices to the new obstacle vertices set
        newObstacleVerticesSet.push_back(newObstacleVertices);
    }

    return;
}