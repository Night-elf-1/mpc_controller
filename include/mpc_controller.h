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
# include "mpc_osqp.h"
# include "kinematic.h"
# include "referencePath.h"

namespace shibo{
    namespace controller{
        class MPC_controller{
            public:
                MPC_controller(int nx, int nu, int np, int nc):NX(nx), NU(nu), NP(np), NC(nc){};
                ~MPC_controller();

                int NX, NU, NP, NC;
                Eigen::Vector2d U(0.01, 0.01);
                Eigen::MatrixXd R = Eigen::MatrixXd::Identity(NU, NU);
                Eigen::MatrixXd RB = Eigen::MatrixXd::Identity(NC * NU, NC * NU);
                Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(NX, NX);
                Eigen::MatrixXd QB = 100 * Eigen::MatrixXd::Identity(NP * NX, NP * NX);

                const double MAX_STEER = 10 * (M_PI / 180);                 // [rad]
                const double MAX_VEL = 0.9;                                 // [m/s]
            public:
                std::vector<double> calculate_linearMPC(Eigen::MatrixXd xref, Eigen::Vector3d inital_x, Eigen::MatrixXd dref, KinematicModel agv_model);
                
                void calculate_linearMPC_new(std::vector<PathPoint> &trajectory, Eigen::Vector3d inital_x, Eigen::MatrixXd dref, KinematicModel agv_model);
            private:
        };
    }
}