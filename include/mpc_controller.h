# include <math.h>
# include <iomanip>
# include <memory>
# include <string>
# include <osqp/osqp.h>

# include "eigen3/Eigen/Core"
# include "common.h"
# include "mpc_osqp.h"
# include "kinematic.h"
# include "referencePath.h"

namespace shibo{
    namespace controller{
        class MPC_controller{
            public:
                MPC_controller(int nx, int nu, int np):NX(nx), NU(nu), NP(np){};
                ~MPC_controller();

                int NX, NU, NP;
                Eigen::MatrixXd R = Eigen::MatrixXd::Identity(NU, NU);
                Eigen::MatrixXd Rd = Eigen::MatrixXd::Identity(NU, NU);
                Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(NX, NX);
                Eigen::MatrixXd Qd = Q;

                const double MAX_STEER = 10 * (M_PI / 180);                 // [rad]
                const double MAX_VEL = 0.9;                                 // [m/s]
            public:
                std::vector<double> calculate_linearMPC(Eigen::MatrixXd xref, Eigen::Vector3d inital_x, Eigen::MatrixXd dref, KinematicModel agv_model);
                
                bool compute_mpc(const VehicleState &vehicle_states, const PathData &ref_data, const ControlCommand &cmd);
            private:
        };
    }
}