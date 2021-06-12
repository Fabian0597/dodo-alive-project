// slip.h
#ifndef SLIP_H
#define SLIP_H
#include <string>
#include <iostream>
#include <stdio.h>
#include <rbdl/rbdl.h>
#include <rbdl/addons/luamodel/luamodel.h>
#include <rbdl/addons/geometry/geometry.h>


#include "arkodeInterface.h"
#include "csvtools.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;


class slip
{
    private:
        double legLenght;
        RigidBodyDynamics::Model slipModel;
    public:
        slip();
        int flightPhase(realtype t, N_Vector y, N_Vector ydot, void *user_data);
        int standPhase(realtype t, N_Vector y, N_Vector ydot, void *user_data);
};

#endif

