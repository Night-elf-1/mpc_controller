#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>

class KinematicModel{
    public:
        KinematicModel(double x, double y, double psi, double v, double L, double dt);
        ~KinematicModel();

        struct kinematic
        {
            double x, y, psi, v, L, dt;
        };
        
        kinematic mykinematic;

        std::vector<double> getstate();

        void updatestate(double accel, double delta_f);

        std::vector<Eigen::MatrixXd> statespace(double ref_delta, double ref_yaw);
    private:

};