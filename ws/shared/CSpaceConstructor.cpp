#include "CSpaceConstructor.h"

/// @brief Checks all configurations for collision
std::unique_ptr<amp::GridCSpace2D> CSpaceConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env){

    //Create the C-Space parameters
    std::size_t x0_cells = 100; 
    std::size_t x1_cells = 100; 
    double m_x0_cells = 100.0;
    double m_x1_cells = 100.0;
    double x0_min = -M_PI; 
    double x0_max = M_PI; 
    double x1_min = -M_PI; 
    double x1_max = M_PI; 

    std::unique_ptr<CSpace> cspace(new CSpace(manipulator, env, x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max));

    //Calculate the step size for each dimension
    double x0_step = (x0_max - x0_min)/m_x0_cells;
    double x1_step = (x1_max - x1_min)/m_x1_cells;

    std::cout << "x0_cells: " << x0_cells << " x1_cells: " << x1_cells << std::endl;
    std::cout << "x0_min: " << x0_min << " x0_max: " << x0_max << std::endl;
    std::cout << "x1_min: " << x1_min << " x1_max: " << x1_max << std::endl << std::endl;

    
    //Check in bounds for theta 1
    for (int i = 0; i < x0_cells; i++){
        //Check in bounds for theta 2
        for (int j = 0; j < x1_cells; j++){

            //Calculate the configuration
            double x0 = x0_min + i*x0_step;
            double x1 = x1_min + j*x1_step;

            //Check if the end effector is in collision
            if (cspace->inCollision(x0, x1)){
                cspace->operator()(i, j) = true;
            }
        }
    }
    

   return cspace;
}