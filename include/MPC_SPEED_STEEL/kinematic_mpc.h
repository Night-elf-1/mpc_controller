#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>
#include <tuple>

class KinematicModel_MPC{
    public:
        double x, y, yaw, v, L, dt;
    public:
        KinematicModel_MPC(double x, double y, double psi, double v, double L, double dt) : x(x), y(y), yaw(yaw), v(v), L(L), dt(dt){};
        ~KinematicModel_MPC(){};

        void updatestate(double accel, double delta_f);

        std::tuple<double, double, double, double> getstate();

        // std::vector<Eigen::MatrixXd> statespace(double ref_delta, double ref_yaw);

};