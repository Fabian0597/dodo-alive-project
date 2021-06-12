// articulatedLeg.cc
#include "articulatedLeg.h"
articulatedLeg *currentInstance;

Eigen::MatrixXd pinv_eigen_based(Eigen::MatrixXd & origin);

articulatedLeg::articulatedLeg(std::string modelpath){
    BOOST_LOG_TRIVIAL(info) << "constructor articulatedLeg";
    legLength = 1;
            
    rbdl_check_api_version (RBDL_API_VERSION);
    std::string fileName(modelpath);

    if (!Addons::LuaModelReadFromFile(fileName.c_str(),&articulatedLegModel)){
        BOOST_LOG_TRIVIAL(error) << "Error loading LuaModel: " << fileName << std::endl;
        abort();
    }
    BOOST_LOG_TRIVIAL(info) << "Model DOF: " << articulatedLegModel.dof_count;
    
    myfile.open ("example.csv");

    //init Vector and Matrix
    BOOST_LOG_TRIVIAL(info) << "init variables";
    int dimensions = 2; // xy no z axis at the moment.
    springStiffness = 6900;
    gravity = VectorNd::Zero(2);
    gravity(1)= -9.81;
    int actuatedJoints = articulatedLegModel.dof_count - dimensions;
    iterations = 10;
    timeStep = 0.001;
    deltaLegLength = 0.0;
    total_error_vel = 0.0;
    q = VectorNd::Zero(articulatedLegModel.dof_count);
    qd = VectorNd::Zero(articulatedLegModel.dof_count);
    qdd = VectorNd::Zero(articulatedLegModel.dof_count);
    tau = VectorNd::Zero(articulatedLegModel.dof_count);
    xDes = VectorNd::Zero(dimensions);
    xDESOld  = VectorNd::Zero(dimensions);
    xDES  = VectorNd::Zero(dimensions);
    total_error = VectorNd::Zero(dimensions); 
    actualComOld = VectorNd::Zero(dimensions);
    xComDotStartStand  = VectorNd::Zero(dimensions);
    xComDotStartFlight =  VectorNd::Zero(dimensions);
    bgMatrix = VectorNd::Zero(articulatedLegModel.dof_count);
    BOOST_LOG_TRIVIAL(debug) << "Think about the size of the jacboian because of last line is zeros if we use 3 axis";
    jacobianCog = MatrixNd::Zero(dimensions, articulatedLegModel.dof_count);
    jacobianS = MatrixNd::Zero(dimensions, articulatedLegModel.dof_count);
    massMatrix = MatrixNd::Zero(articulatedLegModel.dof_count, articulatedLegModel.dof_count);
    Eigen::MatrixXd zeroActuatedMatrix = MatrixNd::Zero(actuatedJoints , dimensions);
    Eigen::MatrixXd identityActuatedMatrix = MatrixNd::Identity(actuatedJoints,actuatedJoints);
    BOOST_LOG_TRIVIAL(info) << "zeroActuatedMatrix:\n" << zeroActuatedMatrix;
    BOOST_LOG_TRIVIAL(info) << "identityActuatedMatrix:\n" << identityActuatedMatrix;
    actuationMatrix.resize(zeroActuatedMatrix.rows(), zeroActuatedMatrix.cols() + identityActuatedMatrix.cols());
    actuationMatrix << MatrixNd::Zero(actuatedJoints , dimensions), MatrixNd::Identity(actuatedJoints,actuatedJoints);
    BOOST_LOG_TRIVIAL(info) << "actuationMatrix:\n" << actuationMatrix;
    
    //test 

    jacobianSDot = MatrixNd::Zero(dimensions, articulatedLegModel.dof_count);
    jacobianSOld = MatrixNd::Zero(dimensions, articulatedLegModel.dof_count);
    jacobianCogDot = MatrixNd::Zero(dimensions, articulatedLegModel.dof_count);
    jacobianCogOld = MatrixNd::Zero(dimensions, articulatedLegModel.dof_count);
    rCogDDot =  VectorNd::Zero(2);
    BOOST_LOG_TRIVIAL(info) << "AFTER DOTS";
    xVector = VectorNd::Zero(2*articulatedLegModel.dof_count);




}

int articulatedLeg::setInteration(int iteration){
    iterations = iteration;
    return 0;
}

int articulatedLeg::setDesPosition(double desired_position){
    desired_xCom = desired_position;
}

int articulatedLeg::setTimeStep(double timeStep){
    timeStep = timeStep;    
    return 0;
}
int articulatedLeg::setInit_q_qd(VectorNd qInit,VectorNd qdInit){
    q = qInit;
    qd = qdInit;
    
    xVector.head(articulatedLegModel.dof_count) = q;
    xVector.tail(articulatedLegModel.dof_count) = qd;
    BOOST_LOG_TRIVIAL(info) << "q:\n" << q;
    BOOST_LOG_TRIVIAL(info) << "qd:\n" << qd;
    CalcCog();
    impactCom = actualCom;
    BOOST_LOG_TRIVIAL(info) << "impactCom:\n" << impactCom;
    return 0;
}

int articulatedLeg::setSpringStiffness(double springStiffness){
    springStiffness = springStiffness;
    return 0;
}

int articulatedLeg::JacobianS(void){
    MatrixNd J3 = MatrixNd::Zero(6, articulatedLegModel.dof_count);
    MatrixNd J2 = MatrixNd::Zero(6, articulatedLegModel.dof_count);
    MatrixNd J4 = MatrixNd::Zero(6, articulatedLegModel.dof_count);
    MatrixNd J5 = MatrixNd::Zero(6, articulatedLegModel.dof_count);
    jacobianSOld = jacobianS;
    CalcPointJacobian(articulatedLegModel, q, articulatedLegModel.GetBodyId("floatingBase"), Vector3dZero    , J2, true);
    
    CalcPointJacobian(articulatedLegModel, q, articulatedLegModel.GetBodyId("foot"), Vector3dZero, J5, true);
    jacobianBaseS =  J5.block(0,0,2, articulatedLegModel.dof_count)-J2.block(0,0,2, articulatedLegModel.dof_count);
    jacobianS =  J5.block(0,0,2, articulatedLegModel.dof_count);
    jacobianSDot = calc_gradient(jacobianS,jacobianSOld,timeStep);
    BOOST_LOG_TRIVIAL(info) << "jacobianS:\n" << jacobianS;
    BOOST_LOG_TRIVIAL(info) << "jacobianSDot:\n" << jacobianSDot; 
    
    
    
    CalcPointJacobian(articulatedLegModel, q, articulatedLegModel.GetBodyId("link1"), Vector3dZero    , J3, true);
    CalcPointJacobian(articulatedLegModel, q, articulatedLegModel.GetBodyId("link2"), Vector3dZero    , J4, true);
    CalcPointJacobian(articulatedLegModel, q, articulatedLegModel.GetBodyId("foot"), Vector3dZero, J5, true);
    MatrixNd J6 = J5.block(0,0,2,4);


    return 0;
}


int articulatedLeg::JacobianStar(void){
    Eigen::MatrixXd actuationMatrixNullspaceS = actuationMatrix*nullspaceS;
    Eigen::MatrixXd jacobianStar2thBracket = actuationMatrixNullspaceS*massMatrix.inverse()*actuationMatrixNullspaceS.transpose();
    jacobianStar = jacobianCog*massMatrix.inverse()*actuationMatrixNullspaceS.transpose()*jacobianStar2thBracket.inverse();
    BOOST_LOG_TRIVIAL(debug) << "jacobianStar:\n" << jacobianStar;
    return 0;
}

int articulatedLeg::CalcCog(void ){
    int num_of_joints = articulatedLegModel.mBodies.size();
    Eigen::MatrixXd MassWightedCog = MatrixNd::Zero(3, 1);
    double mass_sum;
    for(int i = 0; i < num_of_joints; i ++) {
        Eigen::MatrixXd CoGinBody = articulatedLegModel.mBodies[i].mCenterOfMass;
        Eigen::MatrixXd CogInWorld =CalcBodyToBaseCoordinates(articulatedLegModel,q,i,CoGinBody,true);
        MassWightedCog +=  CogInWorld*articulatedLegModel.mBodies[i].mMass;
        mass_sum = mass_sum + articulatedLegModel.mBodies[i].mMass;
    }

    actualCom = (MassWightedCog / mass_sum).block(0,0,2,1);

    BOOST_LOG_TRIVIAL(info) << "actualCom:\n" << actualCom;
    return 0;
}

int articulatedLeg::CalCurrentLegLength(void) {
    Eigen::MatrixXd pos_foot = CalcBodyToBaseCoordinates(articulatedLegModel, q, articulatedLegModel.GetBodyId("foot"), Vector3dZero, true);
    pos_foot = pos_foot.block(0, 0, 2, 1);
    legLength = (actualCom - pos_foot).norm();

    BOOST_LOG_TRIVIAL(info) << "Current leg length:\n" << legLength;
    return 0;
}


int articulatedLeg::CalCurrentLegAngle(void) {
    Eigen::MatrixXd pos_foot = CalcBodyToBaseCoordinates(articulatedLegModel, q, articulatedLegModel.GetBodyId("foot"), Vector3dZero, true);
    pos_foot = pos_foot.block(0, 0, 2, 1);
    double d_x = pos_foot(0) - actualCom(0);
    //if(d_x >= 0){

    //} else{

    //}
    double d_y = pos_foot(1) - actualCom(1);
    legAngle = atan2(abs(d_x), abs(d_y)) * 180 / M_PI;

    BOOST_LOG_TRIVIAL(info) << "Current leg angle:\n" << legAngle;
    return 0;
}


int articulatedLeg::JacobianCog(void ){
    int num_of_joints = articulatedLegModel.mBodies.size();
    jacobianCogOld = jacobianCog;
    Eigen::MatrixXd jacobianCog_i_sum = MatrixNd::Zero(3, articulatedLegModel.dof_count);
    double mass_sum;
     MatrixNd J2 = MatrixNd::Zero(3, articulatedLegModel.dof_count);
    CalcPointJacobian(articulatedLegModel, q, articulatedLegModel.GetBodyId("floatingBase"), Vector3dZero    , J2, true);
    for(int i = 0; i < num_of_joints; i ++) {
        // init jacobian cog of each joint
        Eigen::MatrixXd jacobianCog_i = MatrixNd::Zero(3, articulatedLegModel.dof_count);

        // Calc jacobian cog of the i-th joint
        CalcPointJacobian(articulatedLegModel, q, i, articulatedLegModel.mBodies[i].mCenterOfMass, jacobianCog_i, true);

        // jacobianCog_i * mi
        jacobianCog_i = jacobianCog_i * articulatedLegModel.mBodies[i].mMass;

        mass_sum = mass_sum + articulatedLegModel.mBodies[i].mMass;
        jacobianCog_i_sum = jacobianCog_i_sum + jacobianCog_i;
    }

    // jacobianCog = sum(jacobianCog_i * mi / m)
    jacobianCog = (jacobianCog_i_sum / mass_sum).block(0,0,2, articulatedLegModel.dof_count);//-J2.block(0,0,2,4);
    robotMass = mass_sum;
    jacobianCogDot = calc_gradient(jacobianCog,jacobianCogOld,timeStep);
    BOOST_LOG_TRIVIAL(info) << "jacobianCog:\n" << jacobianCog;
    return 0;
}


int articulatedLeg::LambdaS(void){
    lambdaS = (jacobianS*massMatrix.inverse()*jacobianS.transpose());
    lambdaS = lambdaS.inverse();
    BOOST_LOG_TRIVIAL(info) << "lambdaS:\n" << lambdaS;
    return 0;
}

int articulatedLeg::NullspaceS(void){
    //Eigen::MatrixXd pinv = (currentInstance->jacobianS.transpose()).completeOrthogonalDecomposition().pseudoInverse();
    //Eigen::MatrixXd pinv = massMatrix*jacobianS*massMatrix.inverse();
    //Eigen::MatrixXd nullSpaceMatrixRight =  jacobianS.transpose()*(jacobianS*massMatrix.inverse()*jacobianS.transpose()).inverse()*jacobianS*massMatrix.inverse();
    Eigen::MatrixXd nullSpaceMatrixRight = massMatrix.inverse()*jacobianS.transpose()*lambdaS*jacobianS;
    Eigen::MatrixXd identetyMatrix   = Eigen::MatrixXd::Identity(nullSpaceMatrixRight.rows(),nullSpaceMatrixRight.cols());
    nullspaceS = (identetyMatrix - nullSpaceMatrixRight);
    BOOST_LOG_TRIVIAL(info) << "nullspaceS:\n" << nullspaceS;    
    return 0;
}
    

int articulatedLeg::MassMatrix(void){
    CompositeRigidBodyAlgorithm(articulatedLegModel,q,massMatrix,true);
    massMatrixEE = (jacobianS*massMatrix.inverse()*jacobianS.transpose()).inverse();
    BOOST_LOG_TRIVIAL(info) << "massMatrix:\n" << massMatrix;    
    return 0;
}


int articulatedLeg::MuePStar(void){
    
    Eigen::MatrixXd muePStar1 = lamdaStar*jacobianCog*massMatrix.inverse()*nullspaceS.transpose()*bgMatrix;
    Eigen::MatrixXd mueStar2 = lamdaStar*jacobianCogDot*qd;
    Eigen::MatrixXd mueStar3 = lamdaStar*jacobianCog*massMatrix.inverse()*jacobianS.transpose()*lambdaS*jacobianSDot*qd;
    muePStar = muePStar1-mueStar2+mueStar3;
    BOOST_LOG_TRIVIAL(debug) << "MuePStar:\n" << muePStar;
    return 0;
}

int articulatedLeg::LamdaStar(void){
    Eigen::MatrixXd actuationMatrixNullspaceS =  actuationMatrix*nullspaceS;
    Eigen::MatrixXd lamdaStarTemp = jacobianCog*massMatrix.inverse()*actuationMatrixNullspaceS.transpose()*jacobianStar.transpose();
    lamdaStar = lamdaStarTemp.inverse();  
    BOOST_LOG_TRIVIAL(debug) << "LamdaStar:\n"<< lamdaStar;  
    return 0;
}

int articulatedLeg::SpringForce(void){
    MatrixNd zeroVector3 =  MatrixNd::Zero(3, 1);
    Eigen::MatrixXd footInBase =CalcBodyToBaseCoordinates(articulatedLegModel,q,articulatedLegModel.GetBodyId("foot"),zeroVector3,true).block(0,0,2,1);
    Eigen::MatrixXd impactComInFoot = impactCom - footInBase;
    Eigen::MatrixXd actualComInFoot = actualCom - footInBase;
    BOOST_LOG_TRIVIAL(info)  << "#############impactCom:\n" << impactCom;
    BOOST_LOG_TRIVIAL(info)  << "#############impactComInFoot:\n" << impactComInFoot;
    BOOST_LOG_TRIVIAL(info)  << "#############actualCom:\n" << actualCom;
    BOOST_LOG_TRIVIAL(info)  << "#############actualComInFoot:\n" << actualComInFoot;

    BOOST_LOG_TRIVIAL(info)  << "#############footInBase:\n" << footInBase;
    Eigen::MatrixXd directionVector = actualComInFoot/actualComInFoot.norm();
    BOOST_LOG_TRIVIAL(info)  << "#############directionVector:\n" << directionVector;
    springLegDelta = impactComInFoot.norm() - actualComInFoot.norm() + deltaLegLength;
    springLegForce = springStiffness * (springLegDelta) * (directionVector);
    if(!standOrFlight){
        springLegForce(0)=0;
        springLegForce(1)=0;
    }
    //myfile << springLegForce(0) << "," <<  springLegForce(1) << "," << springLegDelta<< ","  << directionVector(0) << "," << directionVector(1) << "," << springStiffness << "," << deltaLegLength << "," << impactComInFoot(0)<< "," << impactComInFoot(1) << "," << actualComInFoot(0)  << "," << actualComInFoot(1) << "," <<footInBase(0) << "," << footInBase(1) << "," << standOrFlight<<",\n";
    
    //springLegForce = springStiffness*(impactCom - actualCom);
    BOOST_LOG_TRIVIAL(info)  << "#############springLegForce\n" << springLegForce;
    return 0;
}

int articulatedLeg::SpaceControlForce(void){
    rCogDDot = (1.0/robotMass)*(springLegForce+(robotMass*gravity)); 
    spaceControlForce = lamdaStar*rCogDDot+muePStar;
    BOOST_LOG_TRIVIAL(info) << "spaceControlForce:\n" << spaceControlForce;
    return 0;
}

int articulatedLeg::BGMatrix(void){
    VectorNd qdd2 = VectorNd::Zero(articulatedLegModel.dof_count);
    VectorNd qd2 = VectorNd::Zero(articulatedLegModel.dof_count);
    VectorNd bgMatrix2 = VectorNd::Zero(articulatedLegModel.dof_count);;
    InverseDynamics(articulatedLegModel,q,qd2,qdd2,bgMatrix2);
    NonlinearEffects(articulatedLegModel,q,qd,bgMatrix); 
    return 0;
}


int articulatedLeg::flightPhase(realtype t, N_Vector y, N_Vector ydot, void *user_data) {
    // y    =  [q1, q2, qd1, qd2]
    // ydot =  [qd1, qd2, qdd1, qdd2]
    using Eigen::Map;
    VectorNd q = VectorNd::Zero(currentInstance->articulatedLegModel.dof_count);
    VectorNd qd = VectorNd::Zero(currentInstance->articulatedLegModel.dof_count);
    VectorNd qdd = VectorNd::Zero(currentInstance->articulatedLegModel.dof_count);
    VectorNd tau = VectorNd::Zero(currentInstance->articulatedLegModel.dof_count);

    q  = Map<VectorNd>(NV_DATA_S(y),                  currentInstance->articulatedLegModel.dof_count);
    qd = Map<VectorNd>(NV_DATA_S(y)+ currentInstance->articulatedLegModel.dof_count, currentInstance->articulatedLegModel.dof_count);
    MatrixNd zeroVector3 =  MatrixNd::Zero(3, 1);
    /*
    Eigen::MatrixXd footInBase = CalcBodyToBaseCoordinates(currentInstance->articulatedLegModel,q,currentInstance->articulatedLegModel.GetBodyId("foot"),zeroVector3,true).block(0,0,2,1);
    Eigen::MatrixXd floatingBaseInBase = CalcBodyToBaseCoordinates(currentInstance->articulatedLegModel,q,currentInstance->articulatedLegModel.GetBodyId("floatingBase"),zeroVector3,true).block(0,0,2,1);
    currentInstance->xDes(0) = 0.0;
    currentInstance->xDes(1) = 2.1;
    currentInstance->kp = 1.0;
    */
    //double dp = 0.01;
    /*
    currentInstance->xDES = currentInstance->xDes-footInBase;
    currentInstance->xdDes= currentInstance->calc_gradient(currentInstance->xDES,currentInstance->xDESOld,currentInstance->timeStep); 
    currentInstance->xDESOld = currentInstance->xDES;
    currentInstance->xddDes = currentInstance->massMatrixEE.inverse()*(currentInstance->kp*( currentInstance->xDES)-dp*currentInstance->xdDes ); 
    */
   
    //BOOST_LOG_TRIVIAL(info) << "currentInstance->massMatrixEE: \n" << currentInstance->massMatrixEE;
      //BOOST_LOG_TRIVIAL(info) << "footInBase\n" << footInBase;
      //BOOST_LOG_TRIVIAL(info) << "F:\n "<< currentInstance->massMatrixEE*currentInstance->xddDes;
    //BOOST_LOG_TRIVIAL(info) << "currentInstance->jacobianBaseS:\n" << currentInstance->jacobianS;
     //BOOST_LOG_TRIVIAL(info) << "currentInstance->xddDes\n" << currentInstance->xddDes; 
    //**tau = currentInstance->jacobianS.transpose()*currentInstance->massMatrixEE*currentInstance->xddDes;
    Eigen::MatrixXd pinv = currentInstance->jacobianBaseS.completeOrthogonalDecomposition().pseudoInverse(); 
    
    tau = currentInstance->massMatrix*pinv*currentInstance->xddDes;
    
    
    tau(0) = 0.0;  
    tau(1) = 0.0;
    //BOOST_LOG_TRIVIAL(info) << " tau \n " << tau;
    MatrixNd j2 = MatrixNd::Zero(3, currentInstance->articulatedLegModel.dof_count);
    MatrixNd jacobianFloat = MatrixNd::Zero(2, currentInstance->articulatedLegModel.dof_count);
    MatrixNd disForce = MatrixNd::Zero(2,1);
    disForce(0) = 100;
    disForce(1) = 100;
    CalcPointJacobian(currentInstance->articulatedLegModel, currentInstance->q, currentInstance->articulatedLegModel.GetBodyId("foot"), Vector3dZero, j2, true);
    jacobianFloat = j2.block(0,0,2,currentInstance->articulatedLegModel.dof_count);
    //currentInstance->myfile << "jacobianFloat:,\n" << jacobianFloat << ",\ndisForce," << disForce <<",";
    if(currentInstance->setForcesGlob){
        //tau += jacobianFloat.transpose()*disForce;
        currentInstance->setForcesGlob = false;
    }
    ForwardDynamics(currentInstance->articulatedLegModel, q, qd, tau, qdd);
    //BOOST_LOG_TRIVIAL(info) << "qdd\n" << qdd; 
    //qd += pinv*xdot;
    //

    VectorNd res = VectorNd::Zero(2*currentInstance->articulatedLegModel.dof_count);

    res.head(currentInstance->articulatedLegModel.dof_count) = qd;
    res.tail(currentInstance->articulatedLegModel.dof_count) = qdd;

    for(int i = 0; i<2*currentInstance->articulatedLegModel.dof_count; i++) {
        NV_Ith_S(ydot, i) = res[i];
    }
    return 0;
}


int articulatedLeg::standPhase(realtype t, N_Vector y, N_Vector ydot, void *user_data) {
    // y    =  [q1, q2, qd1, qd2]
    // ydot =  [qd1, qd2, qdd1, qdd2]
    using Eigen::Map;
    //currentInstance->updateCalculation();
    
    VectorNd q = VectorNd::Zero(currentInstance->articulatedLegModel.dof_count);
    VectorNd qd = VectorNd::Zero(currentInstance->articulatedLegModel.dof_count);
    VectorNd qdd = VectorNd::Zero(currentInstance->articulatedLegModel.dof_count);
    VectorNd tau =  VectorNd::Zero(currentInstance->articulatedLegModel.dof_count);
    
    q  = Map<VectorNd>(NV_DATA_S(y),                  currentInstance->articulatedLegModel.dof_count);
    qd = Map<VectorNd>(NV_DATA_S(y)+ currentInstance->articulatedLegModel.dof_count, currentInstance->articulatedLegModel.dof_count);
    currentInstance->tau  = currentInstance->jacobianStar.transpose() * currentInstance->spaceControlForce;
    Eigen::MatrixXd qdd1 = (currentInstance->actuationMatrix*currentInstance->nullspaceS).transpose()*currentInstance->tau;
    
    Eigen::MatrixXd qdd2 = currentInstance->nullspaceS.transpose()*currentInstance->bgMatrix;
    BOOST_LOG_TRIVIAL(info) << "qd:\n" << qd;
   BOOST_LOG_TRIVIAL(info) << "currentInstance->qd\n" << currentInstance->qd; 
////    Eigen::MatrixXd qdd3 = currentInstance->jacobianS.transpose()*currentInstance->lambdaS*currentInstance->qd;
    Eigen::MatrixXd qdd3 = currentInstance->jacobianS.transpose()*currentInstance->lambdaS*currentInstance->jacobianSDot*qd;
    //qdd = currentInstance->massMatrix.inverse()*(qdd1 - qdd2 -qdd3);
    BOOST_LOG_TRIVIAL(info) << "qdd";
    qdd = currentInstance->massMatrix.inverse()*(qdd1-qdd2-qdd3); 
    //MatrixNd J = MatrixNd::Zero(2, currentInstance->articulatedLegModel.dof_count);
    //MatrixNd J2 = MatrixNd::Zero(3, currentInstance->articulatedLegModel.dof_count);
    //CalcPointJacobian(currentInstance->articulatedLegModel, currentInstance->q, currentInstance->articulatedLegModel.GetBodyId("link2"), Vector3dZero, J2, true);
    //J = J2.block(0,0,2,currentInstance->articulatedLegModel.dof_count);


    //VectorNd F = VectorNd::Zero(2); F << 10.0, 0.0;
    //tau = currentInstance->nullspaceS*J.transpose()*F;
    //CalcMInvTimesTau(currentInstance->articulatedLegModel, q,tau,qdd,true);
    //BOOST_LOG_TRIVIAL(info) << "qdd+++++++++++++++++++:\n" << qdd;
    /*
    //CalcMInvTimesTau(currentInstance->articulatedLegModel, q,tau,qdd,true);
    //ForwardDynamics(currentInstance->articulatedLegModel, q, qd, tau, qdd);
    
    MatrixNd J = MatrixNd::Zero(2, currentInstance->articulatedLegModel.dof_count);
    MatrixNd J2 = MatrixNd::Zero(3, currentInstance->articulatedLegModel.dof_count);
    CalcPointJacobian(currentInstance->articulatedLegModel, currentInstance->q, currentInstance->articulatedLegModel.GetBodyId("foot"), Vector3dZero, J2, true);
    J = J2.block(0,0,2,currentInstance->articulatedLegModel.dof_count);
    
    Eigen::MatrixXd pinv = currentInstance->jacobianS.completeOrthogonalDecomposition().pseudoInverse();
    //Eigen::MatrixXd pinv2 = pinv_eigen_based(J);
    std::cout << "##################################J = \n" << J << std::endl;
    std::cout << "##################################jacobianS: \n" << currentInstance->jacobianS<< std::endl;
    std::cout << "pinv J.completeOrthogonalDecomposition().pseudoInverse():\n"  << pinv << std::endl;
    //std::cout << "pinv2 pinv_eigen_based: \n " << pinv2 << std::endl;

    //J_full = [J_angular J_linear]
    //F_full = [torque force]
    */
    //Eigen::MatrixXd xdot = VectorNd::Zero(2);
    //Eigen::MatrixXd pinv = currentInstance->jacobianS.completeOrthogonalDecomposition().pseudoInverse();

    //xdot(0) = 0.5;
    //qd += pinv*xdot; 
    
    //tau =  ((currentInstance->actuationMatrix*currentInstance->nullspaceS).transpose()).inverse()*currentInstance->nullspaceS.transpose()*currentInstance->bgMatrix;
    //ForwardDynamics(currentInstance->articulatedLegModel, q, qd, tau, qdd);
    
    
    VectorNd res = VectorNd::Zero(2*currentInstance->articulatedLegModel.dof_count);
    res.head(currentInstance->articulatedLegModel.dof_count) = qd;
    res.tail(currentInstance->articulatedLegModel.dof_count) = qdd;
    for(int i = 0; i<2*currentInstance->articulatedLegModel.dof_count; i++) {
        NV_Ith_S(ydot, i) = res[i];
    }
    return 0;
}

int articulatedLeg::updateCalculation(){
    CalcCog();
    CalCurrentLegLength();
    CalCurrentLegAngle();
    
    SpringForce();
    JacobianS();
    MassMatrix();
    LambdaS();
    NullspaceS();
    JacobianCog();
    JacobianStar();
    LamdaStar();
    BGMatrix();

    MuePStar();

    SpaceControlForce();

    return 0;
}

int articulatedLeg::run(){
    currentInstance = this;
    realtype t = 0.;                                        //Initial time
    realtype dt = timeStep;                                     //Time step

 
    arkodeInterface solverStandPhase(articulatedLeg::standPhase,xVector); 
    arkodeInterface solverFlightPhase(articulatedLeg::flightPhase,xVector);
    //Init containers
    std::vector<std::vector< double > > matrixData;
    std::vector< double > rowData(articulatedLegModel.dof_count+1); //Erster Eintrag für Zeit

    rowData[0] = t;
    for(unsigned int z=0; z < articulatedLegModel.dof_count; z++){
        rowData[z+1] = xVector[z];
    }

    matrixData.push_back(rowData);

    bool impact = false;
    standOrFlight = false; //stand = true ; fight == false;
    bool firstIter = true;
    myfile << "time,actualComX,actualComY,xComDotX,xComDotY,angleOfAttackDeg,xComDifPhase,springLegDelta,Phase,footInBaseX,footInBaseY,deltaLegLength,none\n";
    //myfile << "directionVector(0) ,  directionVector(1) , springStiffness , deltaLegLength ,impactComInFoot(0),impactComInFoot(1) ,<< actualComInFoot(0)  , actualComInFoot(1)\n";
    double lastAngle = 3.0;
     
    double angleOfAttackDeg = 0.0;
    bool init = true;
    bool setForces = true;
    bool setForcesGlob = false;
    int lauf = 0;
    contactNr = 0;
    for(int i = 0; i<iterations; i++) {
        lauf+=1;
        t += dt;

        if(true){
            updateCalculation();
            if(init){
                actualComOld = actualCom;
                init = false;
            }
            if(actualCom(1)<0.0){
                break;
            }
            MatrixNd zeroVector3 =  MatrixNd::Zero(3, 1);

            Eigen::MatrixXd floatingBaseInBase = CalcBodyToBaseCoordinates(articulatedLegModel,q,articulatedLegModel.GetBodyId("floatingBase"),zeroVector3,true).block(0,0,2,1);
            xComDot = calc_gradient(actualCom,actualComOld,timeStep);
            actualComOld = actualCom;
                

            VectorNd x_new = VectorNd::Zero(2*articulatedLegModel.dof_count);  
                        Eigen::MatrixXd footInBase =CalcBodyToBaseCoordinates(articulatedLegModel,q,articulatedLegModel.GetBodyId("foot"),zeroVector3,true).block(0,0,2,1);
            BOOST_LOG_TRIVIAL(info) << "footInBase:\n" << footInBase; 
            try{
            if(footInBase(1) > 0){
                standOrFlight = false; //flight
            }
            else{
                BOOST_LOG_TRIVIAL(info) << "##########################################springLegForce.norm: " <<springLegForce.norm();
                //if(springLegForce(1) < 0.0 && xComDot(1) >= 0.0){
                if(springLegDelta < 0.0 && xComDot(1) > 0.0){
                    standOrFlight = false;//flight
                    impactCom = actualCom;
                   updateCalculation();
 
                }
                else{
                    standOrFlight = true; //stand
                }
            }
              //myfile << t << ","  << xComDot(0) << ","   << xComDot(1) << ","   <<  springLegForce(0) <<     "," << standOrFlight << "," << footInBase(0) << "," << footInBase(1)  <<  ",\n";          
            if(standOrFlight == false){
                //impactCom = actualCom;
 
                /*
                 *POSITION CONTROLLER #############################################################################################################################################
                 */
                double desPos = desired_xCom;
                //double desPos = 4;

                /*
                else if(lauf < 10000){ desPos = 3;}
                else if(lauf < 16000){ desPos = -3;}
                else if(lauf < 24000){ desPos = 10;}
                */
                double maxVel = 1.0;

                double pVel = 0.4;
                double iVelPos = 0.1;
                double errorPos = desPos-actualCom(0);
                total_error_pos += errorPos;
                double maxPos = 1;
                if(total_error_pos > maxPos){total_error_pos = maxPos;}
                if(total_error_pos < -maxPos){total_error_pos = -maxPos;}
                double setVel = pVel*(errorPos) + iVelPos*total_error_pos;

                if(setVel > maxVel){setVel = maxVel;}
                if(setVel < -maxVel){setVel = -maxVel;}
                double xDotDes = setVel;//0.0;
                double error  =  xComDot(0)-xDotDes;//xComDotStartFlight(0);
                /*
                 *DESTURBENT
                 */

               if (abs(xComDot(1)) < 0.01 && contactNr == 3 && setForces )
               {
                    setForces = false;
                    setForcesGlob = true;

               } 
                /*
                 * FIRST IMPACT #################################################################################################################################################
                 */
                if(firstIter){
                        //xVector(6) = 0.0;
                        //xVector(7) = 0.0;
                        firstIter = false;
                    deltaLegLength = 0;
                        /*
                         * Velocity Controller
                         */
                        xComDotStartFlight = xComDot;
                        xComDifPhase = xComDotStartFlight - xComDotStartStand;
                        total_error_vel += error; //accumalates the error - integral term
                }
                /*
                 *VELCOTIY CONTROLLER ###############################################################################################################################################
                 */

                // STABLE HOPPING ON ONE POSE
                //angleOfAttackDeg =  (error/1.5)*21;//21.0; //(xComDifPhase(0)/1.5)*14;
                
                // STABLE FORWARD HOPING 
                //angleOfAttackDeg = lastAngle  + (error/1.5)*5;
                //angleOfAttackDeg = lastAngle  + (-19*abs(xComDot(0))+25)*error;
                
                //JPS controller
                //
                //
                //
                //total_error_vel += error; //accumalates the error - integral term
                double max_control_vel = 6;
                double min_control_vel = -6;
                if (total_error_vel >= max_control_vel) total_error_vel = max_control_vel;
                else if (total_error_vel <= min_control_vel) total_error_vel = min_control_vel;

            //Controller
                //
                //
                double ivel =3.5;// 1.5;
                double c = 5.0;
                double xComDotTemp = xComDot(0);
                //if(abs(xComDotTemp)<0.2){xComDotTemp = 0;}
                double k = 4;//(-19*abs(xComDotTemp)+10);//10
                if(k < 0){k = 0;}
                if(lauf%5 == 0){ angleOfAttackDeg = c * xComDotTemp + k * error + ivel * total_error_vel;}

                if (xComDot(0) > 0){
                    
                }
                if (xComDot(0) < 0){

                }
                if(angleOfAttackDeg > 18){ angleOfAttackDeg = 18;}
                if(angleOfAttackDeg < -18){ angleOfAttackDeg = -18;}
                /*
                if(xComDifPhase(0) > 0)
                {
                    angleOfAttackDeg += 13.0;
                }
                else if(xComDifPhase(0) < 0)
                {   
                    angleOfAttackDeg -= 13.0;
                }
                else{
                    angleOfAttackDeg = angleOfAttackDeg;
                }
                */
            //++++++++++++++{}



                MatrixNd zeroVector3 =  MatrixNd::Zero(3, 1); 
                
                double legLenghtSping = 0.9;
                double angleOfAtack = angleOfAttackDeg * M_PI / 180.0;
                
                /*############################################################################################################################
                 *LEG CONTROLLER
                 */
                xDes(0) = actualCom(0) + sin(angleOfAtack)*legLenghtSping;
                xDes(1) = actualCom(1) - cos(angleOfAtack)*legLenghtSping;
                //P part
                kp = 1020;//1020;//1000//220;
                xDES = xDes-footInBase;
                //D Part  
                double dp = 70.0; //70
                xDES = xDes-footInBase;
                xdDes= calc_gradient(xDES,xDESOld,timeStep); 
                xDESOld = xDES;

                //I Part
                double max_control = 0.5;
                double min_control = -0.5;
                double ki =100;//100
                total_error += xDES; //accumalates the error - integral term
                if (total_error(0) >= max_control) total_error(0) = max_control;
                else if (total_error(0) <= min_control) total_error(0) = min_control;
                if (total_error(1) >= max_control) total_error(1) = max_control;
                else if (total_error(1) <= min_control) total_error(1) = min_control;
                    
                //Controller
                xddDes = massMatrixEE.inverse()*(kp*( xDES)+dp*xdDes+ ki*timeStep*total_error); 

                

                //##########################################################################################################################
                //integrator
                updateCalculation();
 
                x_new = solverFlightPhase.integrate(xVector, dt);
                impact = false;
                
            } //IF
            else{ // GROUND PAHSE
                BOOST_LOG_TRIVIAL(info) << "ground contact";
                if(!impact){  //IMPACT of the foot
                    lastAngle = angleOfAttackDeg;
                    impactCom = actualCom;
                    impact = true;
                    firstIter = true;
                    xComDotStartStand =  xComDot;
                
                    //myfile << impactCom(0) << ","  << impactCom(1) << ",\n";
                    // Energy compensation for impact at landing
                    // Loss of energy of the CoG point in the collision
                    double d_E_kin = 0.5 * robotMass * qd.transpose() *
                        (jacobianCog.transpose() * jacobianCog - nullspaceS.transpose() * jacobianCog.transpose() * jacobianCog * nullspaceS)
                        * qd;
                    //myfile << d_E_kin << "," << robotMass << "," << qd(0)  << "," << qd(1) << "," << qd(2) << "," << qd(3)  << ",\n"; 
                    // spring pre-compressed length
                    int tempSign = 1;
                    if(d_E_kin < 0.0){
                        d_E_kin = abs(d_E_kin);
                        tempSign = -1;
                    }
                    //0.07
                     
                    deltaLegLength =   sqrt(2 * d_E_kin / springStiffness);
                    updateCalculation();
                    //qd = nullspaceS*qd;
                    ///legLenghtOfActualLeg = legLenghtOfActualLeg + deltaLegLength
                    //upadete()
                }
                // new generalized velocity after impact
                //qd = nullspaceS*qd;
                qd = nullspaceS*qd;
                for(int lauf = 0; lauf < articulatedLegModel.dof_count;lauf++){
                    xVector(lauf)= q(lauf); 
                    xVector(articulatedLegModel.dof_count+lauf) = qd(lauf);
                }
                
                x_new = solverStandPhase.integrate(xVector, dt); 
            }
            myfile << t << "," << actualCom(0) << "," << actualCom(1) << "," << xComDot(0) << "," << xComDot(1) << "," << angleOfAttackDeg << "," << xComDifPhase(0) << "," << springLegDelta << "," << standOrFlight << "," << footInBase(0) << "," << footInBase(1) << "," << deltaLegLength << ",\n";
            } //try close
            catch(int error){
                BOOST_LOG_TRIVIAL(error) << "ERROR: " << error << "\nq:" << q; 
            }
            for(int lauf = 0; lauf < articulatedLegModel.dof_count;lauf++){
                q(lauf) = x_new(lauf); 
                qd(lauf) = x_new(articulatedLegModel.dof_count+lauf);
            }
            //qd = nullspaceS*qd;
            BOOST_LOG_TRIVIAL(info) << "q:\n" << q;
            BOOST_LOG_TRIVIAL(info) << "qd:\n" << qd;
            
            xVector = x_new;
        }
        

        rowData[0] = t;
        for(unsigned int z=0; z < articulatedLegModel.dof_count; z++){
            rowData[z+1] = xVector[z];
        }

        matrixData.push_back(rowData);
    }

    myfile.close();
    std::string emptyHeader("");
    std::string fileNameOut("../output/animationArticulatedLegModel.csv");
    printMatrixToFile(matrixData,emptyHeader,fileNameOut);
    printf("Wrote: ../output/animation.csv (meshup animation file)\n");
    
    return 0;
}

realtype articulatedLeg::calc_gradient(realtype x_new, realtype x_old, realtype step_size) {
    // TODO: Check if we actually need to normalize in regards to the size of one step here
    return (x_new - x_old) / step_size; // normalized in regards to the size of a timestep with the assumption that the number of steps is always 1
    // return (x_1 - x_2) // normalized in regards to the number of steps with the assumption that the number of steps is always 1
}


VectorNd articulatedLeg::calc_gradient(VectorNd x_new, VectorNd x_old, realtype step_size) {
    // TODO: Check if we actually need to normalize in regards to the size of one step here
    return (x_new - x_old) / step_size; ; // normalized in regards to the size of a timestep with the assumption that the number of steps is always 1
    // return (x_1 - x_2) // normalized in regards to the number of steps with the assumption that the number of steps is always 1
}
 Eigen::MatrixXd articulatedLeg::calc_gradient(Eigen::MatrixXd x_new, Eigen::MatrixXd x_old, realtype step_size){    return (x_new - x_old) / step_size; ;
}

