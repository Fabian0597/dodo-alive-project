#define BOOST_LOG_DYN_LINK 1

    // articulatedLeg.h
#ifndef ARTICULATEDLEG_H
#define ARTICULATEDLEG_H
#include <string>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <rbdl/rbdl.h>
#include <rbdl/addons/luamodel/luamodel.h>
#include <rbdl/addons/geometry/geometry.h>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <math.h>
#include "arkodeInterface.h"
#include "csvtools.h"
#include <Eigen/SVD>
#include <Eigen/Core>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;


class articulatedLeg 
{
    private:
        bool standOrFlight;
        double deltaLegLength;
        double legLength;
        double legAngle;
        double springStiffness;
        double robotMass;
        double timeStep;
        double springLegDelta;
        int iterations;
        double kp;
        double kv;
        double total_error_vel;
        double total_error_pos;
        double desired_xCom;
        int contactNr;
        bool setForcesGlob;
        RigidBodyDynamics::Model articulatedLegModel;
    public:
        articulatedLeg(std::string modelpath);
        //variables
        Eigen::MatrixXd jacobianS; //DONE
        Eigen::MatrixXd jacobianBaseS;
        Eigen::MatrixXd jacobianSDot;
        Eigen::MatrixXd jacobianSOld; 
        Eigen::MatrixXd jacobianStar; //DONE
        Eigen::MatrixXd jacobianCog; 
        Eigen::MatrixXd jacobianCogDot;
        Eigen::MatrixXd jacobianCogOld;
        Eigen::MatrixXd lambdaS; //DONE
        Eigen::MatrixXd nullspaceS; //DONE
        Eigen::MatrixXd massMatrix; //DONE
        Eigen::MatrixXd massMatrixEE;
        Eigen::MatrixXd actuationMatrix; //DONE
        Eigen::MatrixXd muePStar; //CHECK bmatrx
        Eigen::MatrixXd pStar; //CHECK g
        Eigen::MatrixXd lamdaStar; //DONE
        Eigen::MatrixXd spaceControlForce;
        Eigen::MatrixXd rCogDDot;
        Eigen::MatrixXd springLegForce;
        Eigen::MatrixXd actualCom;
        Eigen::MatrixXd actualComOld;
        Eigen::MatrixXd impactCom;
        
        VectorNd q; 
        VectorNd qd;
        VectorNd qdd; 
        VectorNd tau;
        VectorNd bgMatrix;
        VectorNd gravity;
        VectorNd xVector;
        VectorNd xddDes;
        VectorNd xdDes;
        VectorNd xDes;
        VectorNd xDESOld;
        VectorNd xDES;
        VectorNd total_error;
        VectorNd xComDot;
        VectorNd xComDotStartStand;
        VectorNd xComDotStartFlight;
        VectorNd xComDifPhase;
        std::ofstream myfile;
        //methodes
        
        int setInteration(int);
        int setDesPosition(double);
        int setTimeStep(double);
        int setInit_q_qd(VectorNd qInit,VectorNd qdInit);
        int setSpringStiffness(double);
        int CalcCog();
        int CalCurrentLegLength();
        int CalCurrentLegAngle();
        int SpringForce();
        int BGMatrix();
        int JacobianS(); //DONE
        int JacobianStar(); //DONE 
        int JacobianCog();
        int LambdaS(); //DONE
        int NullspaceS(); //DONE
        int MassMatrix(); //DONE
        int MuePStar(); //CHECK for b
        int LamdaStar(); //DONE
        int SpaceControlForce();
        int run();
        int updateCalculation();
        static int flightPhase(realtype t, N_Vector y, N_Vector ydot, void *user_data);
        static int standPhase(realtype t, N_Vector y, N_Vector ydot, void *user_data);
        realtype calc_gradient(realtype x_new, realtype x_old, realtype step_size);
        VectorNd calc_gradient(VectorNd x_new, VectorNd x_old, realtype step_size);
        Eigen::MatrixXd calc_gradient(Eigen::MatrixXd x_new, Eigen::MatrixXd x_old, realtype step_size); 
};

#endif

