#include "PotentialPlanner.h"

//Temporary Constructor, add one that can use hyperparameters later
PotentialPlanner::PotentialPlanner() {
    //  Attracitve Potential
    zeta = 1;
    d_star = 0.5;

    //  Repulsive Potential
    Q_star = 100;
    eta = 0.1;

    //  Gradient Descent
    epsilon = 0.1;
    stepSize = 0.01;
    maxSteps = 10000;
    
    //  Range Detector
    int rays = 20;
    sensor = RangeDetector(Q_star, rays);
}

PotentialPlanner::PotentialPlanner(double zeta, double d_star, double eta, double Q_star, double epsilon, double stepSize, int maxSteps, int rays) {
    //  Attracitve Potential
    this->zeta = zeta;
    this->d_star = d_star;

    //  Repulsive Potential
    this->Q_star = Q_star;
    this->eta = eta;

    //  Gradient Descent
    this->epsilon = epsilon;
    this->stepSize = stepSize;
    this->maxSteps = maxSteps;

    this->doRawScanning = false;
    this->setIndex = -1;
    
    //  Range Detector
    sensor = RangeDetector(Q_star, rays);
}

PotentialPlanner::PotentialPlanner(double zeta, double d_star, double eta, double Q_star, double epsilon, double stepSize, int maxSteps, int rays, Eigen::Vector2d push_direction, double push_magnitude) {
    //  Attracitve Potential
    this->zeta = zeta;
    this->d_star = d_star;

    //  Repulsive Potential
    this->Q_star = Q_star;
    this->eta = eta;

    //  Gradient Descent
    this->epsilon = epsilon;
    this->stepSize = stepSize;
    this->maxSteps = maxSteps;
    this->goalRange = 0.5;

    //  Push Potential
    this->push_direction = push_direction;
    this->push_magnitude = push_magnitude;

    this->doRawScanning = false;

    this->setIndex = -1;
    
    //  Range Detector
    sensor = RangeDetector(Q_star, rays);
}

PotentialPlanner::PotentialPlanner(std::vector<GDVars> varSets) {
    // //  Attracitve Potential
    // this->zeta = varSets[0].zeta;
    // this->d_star = varSets[0].d_star;

    // //  Repulsive Potential
    // this->Q_star = varSets[0].Q_star;
    // this->eta = varSets[0].eta;

    // //  Gradient Descent
    // this->epsilon = varSets[0].epsilon;
    // this->stepSize = varSets[0].stepSize;
    // this->maxSteps = varSets[0].maxSteps;
    // this->goalRange = 0.5;

    // //  Push Potential
    // this->push_direction = varSets[0].push_direction;
    // this->push_magnitude = varSets[0].push_magnitude;

    // this->doRawScanning = false;
    
    // //  Range Detector
    // sensor = RangeDetector(Q_star, varSets[0].rays);

    this->setIndex = 0;
    this->varSets = varSets;
}

amp::Path2D PotentialPlanner::plan(const amp::Problem2D& problem) {
    //Check if the plan needs to change for grading
    if (setIndex != -1 && setIndex < varSets.size()){
        changePlan();
    }

    amp::Path2D path;
    //Add the initial point to the path
    path.waypoints.push_back(problem.q_init);
    int i = 0; //Counter for the number of iterations

    //Starting Conditions and Variables
    Eigen::Vector2d q = problem.q_init; //Current point
    Eigen::Vector2d q_goal = problem.q_goal; //Goal point
    Eigen::Vector2d GU; //Potential Function

    //Debug: Output if the planner is in raw sensing mode
    if (doRawScanning){
        std::cout << "########## Raw Sensing Mode ##############" << std::endl;
    }

    do {
        //Scan the current point to get the obstacles in range
        if (doRawScanning){
            //std::cout<< "Debug:: Got to this point" << std::endl;
            sensor.scanRaw(problem.obstacles, q);
            sensor.avgDirection();
        }else{
            sensor.scan(problem.obstacles, q);
        }
        //Calculate the potential function
        GU = attractivePotential(q, q_goal) + repulsivePotential(q, q_goal, problem.obstacles);

        //Check if there is an initial push
        if (i == 0) {
            GU = GU - push_magnitude * push_direction;
        }

        //std::cout << "GU: (" << GU.x() << ", " << GU.y() << ")" << std::endl;

        //Take a step
        q = q - stepSize * GU;

        //Check if q is nan or inf
        if (q!= q || q.x() == INFINITY || q.y() == INFINITY || q.x() == -INFINITY || q.y() == -INFINITY){
            return path;
        }

        if ((q-q_goal).norm() < goalRange) {
            path.waypoints.push_back(q_goal);
            return path;
        }

        //Add the new point to the path
        path.waypoints.push_back(q);
        i++;
    }
    while (GU.norm() > epsilon && i < maxSteps); 
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

void PotentialPlanner::changeSensingMode() {
    doRawScanning = !doRawScanning;
}

GDVars::GDVars(double zeta, double d_star, double eta, double Q_star, double epsilon, double stepSize, int maxSteps, int rays, bool rawScanning) {
    this->zeta = zeta;
    this->d_star = d_star;
    this->eta = eta;
    this->Q_star = Q_star;
    this->epsilon = epsilon;
    this->stepSize = stepSize;
    this->maxSteps = maxSteps;
    this->rays = rays;
    this->doRawScanning = rawScanning;
}

void PotentialPlanner::changePlan(){
    //  Attracitve Potential
    this->zeta = varSets[setIndex].zeta;
    this->d_star = varSets[setIndex].d_star;

    //  Repulsive Potential
    this->Q_star = varSets[setIndex].Q_star;
    this->eta = varSets[setIndex].eta;

    //  Gradient Descent
    this->epsilon = varSets[setIndex].epsilon;
    this->stepSize = varSets[setIndex].stepSize;
    this->maxSteps = varSets[setIndex].maxSteps;
    this->goalRange = 0.5;

    this->doRawScanning = varSets[setIndex].doRawScanning;
    
    //  Range Detector
    sensor = RangeDetector(Q_star, varSets[setIndex].rays);

    //Print out the new changes to variables
    std::cout << "########## New Plan ##############" << std::endl;
    std::cout << "zeta: " << zeta << std::endl;
    std::cout << "d_star: " << d_star << std::endl;
    std::cout << "eta: " << eta << std::endl;
    std::cout << "Q_star: " << Q_star << std::endl;
    std::cout << "epsilon: " << epsilon << std::endl;
    std::cout << "stepSize: " << stepSize << std::endl;
    std::cout << "maxSteps: " << maxSteps << std::endl;
    std::cout << "rays: " << varSets[setIndex].rays << std::endl;

    

    //Change to next index
    setIndex++;
}