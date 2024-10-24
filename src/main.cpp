#include "mpc_controller.h"

int main(int argc, char const *argv[])
{
    double end_x = 50, end_y = 50;
    int min_ind;
    bool finish_ = true;
    parameters param;
    //mpc_controller(param.NX, param.NU, param.NP);
    shibo::controller::MPC_controller mpc(param.NX, param.NU, param.NP, param.NC);
    //shibo::controller::MPC_controller(param.NX, param.NU, param.NP);            // 初始化mpc参数

    Eigen::VectorXd initial_x(param.NX);                                        // 初始化agv初始状态 x y yaw
    initial_x << 0.0, 0.0, 0.0;

    double dt = 0.1;
    double L = 3.7;
    KinematicModel agv(initial_x(0), initial_x(1), initial_x(2), 0.7, L, dt);   // 初始化运动学模型 x y yaw v L dt

    std::vector<double> agv_state = {0.0, 0.0, 0.0, 0.0};                       // 起始点
    std::vector<PathPoint> trajectory;
    MyReferencePath refpath(initial_x, end_x, end_y, trajectory);

    std::vector<double> x_history, y_history;

    while ( finish_ )
    {
        auto lateral_and_index = refpath.calcNearestIndexAndLateralError(initial_x(0), initial_x(1), trajectory);
        std::cout << "最近点索引: " << lateral_and_index.second << std::endl;
        std::cout << "横向距离: " << lateral_and_index.first << std::endl;

        auto [v_real, delta_real] = mpc.calculate_linearMPC_new(trajectory, initial_x, lateral_and_index.second, lateral_and_index.first, agv);

        agv.updatestate(v_real, delta_real);
        x_history.push_back(agv.getstate()[0]);
        y_history.push_back(agv.getstate()[1]);

        const auto state = agv.getstate();
        initial_x << state[0], state[1], state[2];

        if(lateral_and_index.second < trajectory.size() - 1){
            continue;
        }else{
            finish_ = false;
        }
    }

    return 0;
}
