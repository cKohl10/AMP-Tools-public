#include "MyBugAlgorithm.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) const {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;
    //path.waypoints.push_back(problem.q_init);
    //path.waypoints.push_back(Eigen::Vector2d(1.0, 10.0));
    //path.waypoints.push_back(Eigen::Vector2d(3.0, 9.0));
    //path.waypoints.push_back(problem.q_goal);

    // Print all obstacles and their verticies
    for (amp::Obstacle2D o : problem.obstacles){
        o.print();
    }

    return path;
}
