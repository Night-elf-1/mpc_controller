#ifndef MODELPREDICTIVECONTROL_H
#define MODELPREDICTIVECONTROL_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include "cubic_spline.hpp"
#include "OsqpEigen/OsqpEigen.h"
#include "eigen3/Eigen/Core"
#include "common.h"

#define YAW_P2P(angle) fmod(fmod((angle)+M_PI, 2*M_PI)-2*M_PI, 2*M_PI)+M_PI


class MPC_controller{
    public:
        int NX, NU, NP, NC;
                
        Eigen::VectorXd U;
        Eigen::MatrixXd R;
        Eigen::MatrixXd RB;
        Eigen::MatrixXd Q;
        Eigen::MatrixXd QB;

        const double MAX_STEER = 10 * (M_PI / 180);                 // [rad]
        const double MAX_VEL = 0.9;                                 // [m/s]
    public:
        MPC_controller(int nx, int nu, int np, int nc) : NX(nx), NU(nu), NP(np), NC(nc), R(Eigen::MatrixXd::Identity(nu, nu)), RB(Eigen::MatrixXd::Identity(nc * nu, nc * nu)), Q(Eigen::MatrixXd::Identity(nx, nx)), QB(Eigen::MatrixXd::Identity(np * nx, np * nx)), U(Eigen::VectorXd::Constant(nu, 0.01)) {};
        ~MPC_controller(){};

        vector<double> calc_speed_profile(vector<double> rx, vector<double> ry, vector<double> ryaw, double target_speed);
};



#endif // !MODELPREDICTIVECONTROL_H
