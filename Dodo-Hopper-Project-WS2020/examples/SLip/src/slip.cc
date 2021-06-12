// slip.cc
#include "slip.h"
slip::slip(void){
    std::cout << "constructor" << std::endl;
    legLenght = 1;

    rbdl_check_api_version (RBDL_API_VERSION);
    std::string fileName("../model/slip.lua");

    if (!Addons::LuaModelReadFromFile(fileName.c_str(),&slipModel)){
        std::cerr << "Error loading LuaModel: " << fileName << std::endl;
        abort();
    }

}

int slip::flightPhase(realtype t, N_Vector y, N_Vector ydot, void *user_data) {
    // y    =  [q1, q2, qd1, qd2]
    // ydot =  [qd1, qd2, qdd1, qdd2]
    using Eigen::Map;
    std::cout << "FLIGHT" << std::endl;
    VectorNd q = VectorNd::Zero(slipModel.dof_count);
    VectorNd qd = VectorNd::Zero(slipModel.dof_count);
    VectorNd qdd = VectorNd::Zero(slipModel.dof_count);
    VectorNd tau = VectorNd::Zero(slipModel.dof_count);

    q  = Map<VectorNd>(NV_DATA_S(y),                  slipModel.dof_count);
    qd = Map<VectorNd>(NV_DATA_S(y)+ slipModel.dof_count, slipModel.dof_count);



    ForwardDynamics(slipModel, q, qd, tau, qdd);

    VectorNd res = VectorNd::Zero(2*slipModel.dof_count);

    res.head(slipModel.dof_count) = qd;
    res.tail(slipModel.dof_count) = qdd;

    for(int i = 0; i<2*slipModel.dof_count; i++) {
        NV_Ith_S(ydot, i) = res[i];
    }
    return 0;
}
int slip::standPhase(realtype t, N_Vector y, N_Vector ydot, void *user_data) {
    // y    =  [q1, q2, qd1, qd2]
    // ydot =  [qd1, qd2, qdd1, qdd2]
    std::cout << "HALEJULIA###############" << std::endl;
    using Eigen::Map;

    VectorNd q = VectorNd::Zero(slipModel.dof_count);
    VectorNd qd = VectorNd::Zero(slipModel.dof_count);
    VectorNd qdd = VectorNd::Zero(slipModel.dof_count);
    VectorNd tau = VectorNd::Zero(slipModel.dof_count);

    q  = Map<VectorNd>(NV_DATA_S(y),                  slipModel.dof_count);
    qd = Map<VectorNd>(NV_DATA_S(y)+ slipModel.dof_count, slipModel.dof_count);



    ForwardDynamics(slipModel, q, qd, tau, qdd);
    double k = 70.0;
    double mass = 1;
    qdd[1] += k*(1-q[1]);
    VectorNd res = VectorNd::Zero(2*slipModel.dof_count);

    res.head(slipModel.dof_count) = qd;
    res.tail(slipModel.dof_count) = qdd;
    for(int i = 0; i<2*slipModel.dof_count; i++) {
        NV_Ith_S(ydot, i) = res[i];
    }
    return 0;
}

