#include "mpc_controller.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main(int argc, char const *argv[])
{
    double end_x = 50, end_y = 50;
    int min_ind;
    bool finish_ = true;
    parameters param;

    shibo::controller::MPC_controller mpc(param.NX, param.NU, param.NP, param.NC);

    Eigen::Vector3d initial_x;                                        // 初始化agv初始状态 x y yaw
    initial_x << 0.0, 0.0, 0.0;

    double dt = 0.1;
    double L = 3.7;
    KinematicModel agv(initial_x(0), initial_x(1), initial_x(2), 0.0, L, dt);   // 初始化运动学模型 x y yaw v L dt

    std::vector<double> agv_state = {0.0, 3.0, 0.0, 0.0};                       // 起始点
    std::vector<PathPoint> trajectory;
    MyReferencePath refpath(initial_x, end_x, end_y, trajectory);

    std::vector<double> x, y;
    for (int i = 0; i < trajectory.size(); i++)
    {
        x.push_back(trajectory[i].x);
        y.push_back(trajectory[i].y);
    }

    std::vector<double> x_history, y_history;

    int count = 0;

    plt::figure_size(800, 600);  // 设置窗口大小
    plt::plot(x, y, "b--");      // 绘制参考路径

    while ( finish_ )
    {
        auto lateral_and_index = refpath.calcNearestIndexAndLateralError(initial_x(0), initial_x(1), trajectory);

        auto [v_real, delta_real] = mpc.calculate_linearMPC_new(trajectory, initial_x, lateral_and_index.second, lateral_and_index.first, agv);

        agv.updatestate(v_real, delta_real);
        x_history.push_back(agv.getstate()[0]);
        y_history.push_back(agv.getstate()[1]);

        const auto state = agv.getstate();
        initial_x << state[0], state[1], state[2];

        // cout << "实时的坐标点 X = " << initial_x(0) << "  y = "<< initial_x(1) << "  最近点索引 = " << lateral_and_index.second << "  横向距离 = " << lateral_and_index.first << "  真实控制量v = " << v_real << "  delta = " << delta_real << endl;

        // 更新绘图
        plt::clf();                // 清除当前图像
        plt::plot(x, y, "b--");    // 绘制参考路径
        plt::plot(x_history, y_history, "r-");  // 绘制AGV的实际轨迹

        // plt::scatter({initial_x(0)}, {initial_x(1)}, 20.0, {{"color", "green"}}); // 当前AGV位置
        plt::scatter(std::vector<double>{initial_x(0)}, std::vector<double>{initial_x(1)}, 20.0, {{"color", "green"}});
        plt::pause(0.0001);         // 暂停以更新图形

        if(lateral_and_index.second < trajectory.size() - 1){
            // count += 1;
            continue;
        }else{
            finish_ = false;
        }
    }

    plt::show();  // 显示最终结果

    return 0;
}
