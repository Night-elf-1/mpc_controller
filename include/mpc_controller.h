# include <math.h>
# include <iomanip>
# include <memory>
# include <string>
# include <osqp/osqp.h>
# include <Eigen/Dense>
# include <vector>

#include "OsqpEigen/OsqpEigen.h"
# include "eigen3/Eigen/Core"
# include "common.h"
# include "kinematic.h"
# include "referencePath.h"

namespace shibo{
    namespace controller{
        class MPC_controller{
            public:
                MPC_controller(int nx, int nu, int np, int nc):NX(nx), NU(nu), NP(np), NC(nc), R(Eigen::MatrixXd::Identity(nu, nu)), RB(Eigen::MatrixXd::Identity(nc * nu, nc * nu)), Q(Eigen::MatrixXd::Identity(nx, nx)), QB(100 * Eigen::MatrixXd::Identity(np * nx, np * nx)), U(Eigen::VectorXd::Constant(nu, 0.01)){};
                ~MPC_controller(){};

                int NX, NU, NP, NC;
                
                Eigen::VectorXd U;
                Eigen::MatrixXd R;
                Eigen::MatrixXd RB;
                Eigen::MatrixXd Q;
                Eigen::MatrixXd QB;

                const double MAX_STEER = 10 * (M_PI / 180);                 // [rad]
                const double MAX_VEL = 0.9;                                 // [m/s]
            public:
                std::tuple<double, double> calculate_linearMPC_new(std::vector<PathPoint> &trajectory, Eigen::Vector3d inital_x, int min_index, double min_errors, KinematicModel agv_model);
            private:
        };
    }
}