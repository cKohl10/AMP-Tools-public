#pragma once

#include "AMPCore.h"
#include "LinkMan.h"
#include "CSpace.h"
#include "hw/HW4.h"

class CSpaceConstructor : public amp::GridCSpace2DConstructor{
    public:
        CSpace returnCSpace(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env);

        /// @brief Checks all configurations for collision
        std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env);
};

