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
#include "kinematic_mpc.h"

#define YAW_P2P(angle) fmod(fmod((angle)+M_PI, 2*M_PI)-2*M_PI, 2*M_PI)+M_PI
static inline bool finish = true;
static inline int N_IND_SEARCH = 10;
static inline int target_ind = 0;
static inline double goal_dis = 5.0;

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
        MPC_controller(int nx, int nu, int np, int nc) : NX(nx), NU(nu), NP(np), NC(nc), R(Eigen::MatrixXd::Identity(nu, nu)), RB(Eigen::MatrixXd::Identity(nc * nu, nc * nu)), Q(Eigen::MatrixXd::Identity(nx, nx)), QB(100*Eigen::MatrixXd::Identity(np * nx, np * nx)), U(Eigen::VectorXd::Constant(nu, 0.01)) {};
        ~MPC_controller(){};

        std::vector<double> calculateReferenceSpeeds(const std::vector<double>& curvatures, const double& max_speed);

        vector<double> calc_speed_profile(vector<double> rx, vector<double> ry, vector<double> ryaw, double target_speed);

        std::tuple<int, double> calc_nearest_index(double current_X, double current_y, vector<double> cx, vector<double> cy, vector<double> cyaw, int pind);

        void smooth_yaw(vector<double>& cyaw);

        std::tuple<int, double> calc_ref_trajectory(double current_X, double current_y, vector<double> cx, vector<double> cy, vector<double> cyaw, int& target_ind);

        std::tuple<double, double> mpc_solve(vector<double>& cx, vector<double>& cy, vector<double>& cyaw, vector<double>& ck, vector<double>& speed, Eigen::Vector3d inital_x, int min_index, double min_errors, KinematicModel_MPC agv_model);

        std::tuple<double, int, int> calculate_distance(const double& end_x, const double& end_y, const double& current_x, const double& current_y, const double& r_yaw);
};



#endif // !MODELPREDICTIVECONTROL_H
