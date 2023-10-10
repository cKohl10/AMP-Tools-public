#include "PotentialPlanner.h"

//Temporary Constructor, add one that can use hyperparameters later
PotentialPlanner::PotentialPlanner() {
    //  Attracitve Potential
    zeta = 1;
    d_star = 0.5;

    //  Repulsive Potential
    Q_star = 2;
    eta = 0.1;

    //  Gradient Descent
    epsilon = 0.01;
    stepSize = 0.01;
    maxSteps = 10000;
    
    //  Range Detector
    int rays = 20;
    sensor = RangeDetector(Q_star, rays);
}

amp::Path2D PotentialPlanner::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    //Add the initial point to the path
    path.waypoints.push_back(problem.q_init);
    int i = 0; //Counter for the number of iterations

    //Starting Conditions and Variables
    Eigen::Vector2d q = problem.q_init; //Current point
    Eigen::Vector2d q_goal = problem.q_goal; //Goal point
    Eigen::Vector2d GU; //Potential Function

    do {
        //Scan the current point to get the obstacles in range
        sensor.scan(problem.obstacles, q);

        //Calculate the potential function
        GU = attractivePotential(q, q_goal) + repulsivePotential(q, q_goal, problem.obstacles);
        std::cout << "GU: (" << GU.x() << ", " << GU.y() << ")" << std::endl;

        //Take a step
        q = q - stepSize * GU;

        //Check if q is nan or inf
        if (q!= q || q.x() == INFINITY || q.y() == INFINITY || q.x() == -INFINITY || q.y() == -INFINITY){
            return path;
        }

        //Add the new point to the path
        path.waypoints.push_back(q);
        i++;
    }
    while (GU.norm() > epsilon || i < maxSteps); 
    //Calculate the gradient of the potential function

    return path;

}

// Attractive Potential Function Gradient
Eigen::Vector2d PotentialPlanner::attractivePotential(const Eigen::Vector2d& q, const Eigen::Vector2d& q_goal) {
    Eigen::Vector2d U_att;
    if ((q-q_goal).norm() <= d_star) {
        U_att = zeta * (q-q_goal);
    }
    else {
        U_att = (d_star * zeta * (q-q_goal))/ ((q-q_goal).norm());
    }
    return U_att;
}

// Repulsive Potential Function Gradient
// Returns the sum of all repulsive gradients from each obstacle in range
Eigen::Vector2d PotentialPlanner::repulsivePotential(const Eigen::Vector2d& q, const Eigen::Vector2d& q_goal, const std::vector<amp::Polygon>& obstacles) {
    Eigen::Vector2d U_rep = Eigen::Vector2d(0.0, 0.0);
    std::vector<CastHit> hits = sensor.getLog(); //hitlog from sensor object
    double d_i = 0;
    Eigen::Vector2d dir; //Direction of potential gradient

    for (int i = 0; i < sensor.getLog().size(); i++) {
        d_i = hits[i].distance;
        dir = -hits[i].direction; //Direction of potential gradient
        U_rep = U_rep + (eta * (1/Q_star - 1/d_i) * (1/(d_i * d_i))*dir);
    }
    
    return U_rep;
}