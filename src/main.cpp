#include "mpc_controller.h"

int main(int argc, char const *argv[])
{
    parameters param;
    //mpc_controller(param.NX, param.NU, param.NP);
    shibo::controller::MPC_controller mpc(param.NX, param.NU, param.NP);
    //shibo::controller::MPC_controller(param.NX, param.NU, param.NP);            // 初始化mpc参数

    Eigen::VectorXd initial_x(param.NX);                                        // 初始化agv初始状态 x y yaw
    initial_x << 0.0, 0.0, 0.0;

    double dt = 0.1;
    double L = 3.7;
    KinematicModel agv(initial_x(0), initial_x(1), initial_x(2), 0.7, L, dt);   // 初始化运动学模型

    std::vector<double> agv_state = {0.0, 0.0, 0.0, 0.0};
    MyReferencePath refpath;
    refTraj referenceTrajectory = refpath.CalRefTrajectory(agv_state, param, 1.0);

    std::vector<double> x_history, y_history;

    for (int i = 0; i < param.NP; ++i)
    {
        Eigen::MatrixXd xref = referenceTrajectory.xref;
        Eigen::VectorXd xref_i = xref.col(i);
        Eigen::VectorXd ref_delata = referenceTrajectory.dref.col(i);

        std::vector<double> control_result = mpc.calculate_linearMPC(xref_i, initial_x, ref_delata, agv);

        agv.updatestate(control_result[0], control_result[1]);

        x_history.push_back(agv.getstate()[0]);             // 存储行驶过的路径点
        y_history.push_back(agv.getstate()[1]);

        const auto state = agv.getstate();
        initial_x << state[0], state[1], state[2];
    }
    
    

    return 0;
}
