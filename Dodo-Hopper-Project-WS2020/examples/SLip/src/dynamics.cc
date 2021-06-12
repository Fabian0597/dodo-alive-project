#define BOOST_LOG_DYN_LINK 1


#include <string>
#include <iostream>
#include <stdio.h> 
#include <rbdl/rbdl.h>
#include <rbdl/addons/luamodel/luamodel.h>
#include <rbdl/addons/geometry/geometry.h>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include "csvtools.h"
#include "arkodeInterface.h"
#include "slip.h"
#include "articulatedLeg.h"


using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

RigidBodyDynamics::Model model;

bool applyForce = false;

int flightPhase(realtype t, N_Vector y, N_Vector ydot, void *user_data);

realtype calc_gradient(realtype x_new, realtype x_old, realtype step_size) {
    // TODO: Check if we actually need to normalize in regards to the size of one step here
    return (x_new - x_old) / step_size; // normalized in regards to the size of a timestep with the assumption that the number of steps is always 1
    // return (x_1 - x_2) // normalized in regards to the number of steps with the assumption that the number of steps is always 1
}


VectorNd calc_gradient(VectorNd x_new, VectorNd x_old, realtype step_size) {
    // TODO: Check if we actually need to normalize in regards to the size of one step here
    return (x_new - x_old) / step_size; ; // normalized in regards to the size of a timestep with the assumption that the number of steps is always 1
    // return (x_1 - x_2) // normalized in regards to the number of steps with the assumption that the number of steps is always 1
}

int main (int argc, char* argv[]) {
    BOOST_LOG_TRIVIAL(info) << "Start main function";
    std::cout << "Have " << argc << " arguments:" << std::endl;
    int iterations = 100;
    double desired_position = 0.0;
    if (argc > 1)
    {
      for (int count = 1; count < argc; count++)
    {
      printf("argv[%d] = %s\n", count, argv[count]);
    }
      iterations = atoi(argv[1]);
      desired_position = atof(argv[2]);
    }
    else
    {
      printf("The command had no other arguments.\n");
    } 
    
    double timeStep  = 0.001;
    VectorNd q  = VectorNd::Zero(4);
    VectorNd qd  = VectorNd::Zero(4);
    double j1 = 65 * (M_PI/180.0); //65
    double j2 = -120 * (M_PI/180.0); //-120
    /*
     * q : The model's initial position (x_cog, y_cog) and angles between links (J1, J2)
     * qd: The model's initial velocity.
     * These two parameters should be changed according to the desired initial configuration.
     */
    q << 0,1.5,j1,j2;//1.5
    qd << 0.0,0,0,0;


    std::string modelpath = "../model/articulatedLeg.lua"; 
    articulatedLeg n(modelpath);
    n.setInteration(iterations);
    n.setDesPosition(desired_position);
    n.setTimeStep(timeStep);
    n.setInit_q_qd(q,qd);
    n.run(); 
    //arkodeInterface solverStandPhase(n.standPhase, n.xVector);

    std::string fileName("../model/articulatedLeg3.lua");

    if (!Addons::LuaModelReadFromFile(fileName.c_str(),&model)){
        std::cerr << "Error loading LuaModel: " << fileName << std::endl;
        abort();
    }

    //Dont know why but we need that 
    VectorNd x = VectorNd::Zero(2*model.dof_count);
    arkodeInterface solverFlightPhase(flightPhase, x);
    //Dont Know end




    //Integration loop
    




    return 0;

}


int flightPhase(realtype t, N_Vector y, N_Vector ydot, void *user_data) {
return 0;
}

