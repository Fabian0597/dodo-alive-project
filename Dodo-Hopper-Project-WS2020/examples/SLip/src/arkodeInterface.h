#pragma once

//RBDL
#include <rbdl/rbdl.h>


//SUNDIALS
#include <arkode/arkode_erkstep.h>    // prototypes for ERKStep fcts., consts
#include <nvector/nvector_serial.h>   // serial N_Vector types, fcts., macros
#include <sundials/sundials_types.h>  // def. of type 'realtype'
#include <sundials/sundials_math.h>   // def. of SUNRsqrt, etc.

using RigidBodyDynamics::Math::VectorNd;

// xdot = rhs(x, p)
typedef int(*rhs)(realtype t, N_Vector y, N_Vector ydot, void *user_data);  //dynamics

class arkodeInterface
{
public:
    arkodeInterface(rhs f_, VectorNd x0);
    ~arkodeInterface();
    VectorNd integrate(VectorNd x0, realtype t_step);

private:
    void reInit();

    int flag;
    void *mem;
    N_Vector y;
    realtype t_0;
    rhs f;

    realtype absTolVal;
    realtype relTolVal;

    int check_flag(void *flagvalue, const char *funcname, int opt);


};
