#include "arkodeInterface.h"

arkodeInterface::arkodeInterface(rhs f_, VectorNd x0) {

    f = f_;

    absTolVal = 1e-8;
    relTolVal = 1e-10;

    //Init solution vector
    y = N_VNew_Serial(x0.rows());
    if (check_flag((void *)y, "N_VNew_Serial", 0)) {
        std::cerr << "Error. Could not create New_Serial vector" << std::endl;
    };

    for(int i = 0; i< x0.rows(); i++) {
        NV_Ith_S(y,i) = x0[i];
    }

    //Init time (will be overwritten later)
    t_0 = 0.0;

    //Init memory
    mem = ERKStepCreate(f, t_0, y);
    if (check_flag((void *)mem, "ERKStepCreate", 0)) {
        std::cerr << "Error. Could not initialize memory." << std::endl;
    }


    //Set tolerances
    flag = ERKStepSStolerances(mem, relTolVal, absTolVal);  // Specify tolerances

}
arkodeInterface::~arkodeInterface() {
    N_VDestroy(y);        // Free y vector
    ERKStepFree(&mem);    // Free integrator memory
}


VectorNd arkodeInterface::integrate(VectorNd x0, realtype t_step) {

    using Eigen::Map;
    VectorNd x_res =  VectorNd::Zero(x0.rows());
    realtype t = 0.0;

    for(int i = 0; i< x0.rows(); i++) {
        NV_Ith_S(y,i) = x0[i];
    }


    ERKStepReInit(mem, f, 0.0, y);

    flag = ERKStepSetStopTime(mem, t_step);
    flag = ERKStepEvolve(mem, 1.0, y, &t, ARK_NORMAL);     // call integrator

    if(flag == ARK_TSTOP_RETURN) {
        x_res =  Eigen::Map<VectorNd>(NV_DATA_S(y), x0.rows());
        return x_res;
    }
    else {
        std::cerr << "Error in integration!" << std::endl;
        abort();
    }

}




int arkodeInterface::check_flag(void *flagvalue, const char *funcname, int opt) {
    int *errflag;

    // Check if SUNDIALS function returned NULL pointer - no memory allocated
    if (opt == 0 && flagvalue == NULL) {
        fprintf(stderr, "\nSUNDIALS_ERROR: %s() failed - returned NULL pointer\n\n",
                funcname);
        return 1; }

    // Check if flag < 0
    else if (opt == 1) {
        errflag = (int *) flagvalue;
        if (*errflag < 0) {
            fprintf(stderr, "\nSUNDIALS_ERROR: %s() failed with flag = %d\n\n",
                    funcname, *errflag);
            return 1; }}

    // Check if function returned NULL pointer - no memory allocated
    else if (opt == 2 && flagvalue == NULL) {
        fprintf(stderr, "\nMEMORY_ERROR: %s() failed - returned NULL pointer\n\n",
                funcname);
        return 1; }

    return 0;
}
