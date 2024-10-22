#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "mpc_control.h"  // 假设你已实现mpc_control函数
#include "updateState.h"  // 假设你已实现updateState函数

// 定义 PI 常量
const double PI = 3.14159265359;

int main() {
    // 读取参考路径点
    std::vector<Eigen::Vector2d> refPos = loadPath();  // 假设路径点由loadPath函数加载
    std::vector<double> refPos_x, refPos_y;
    for (const auto& point : refPos) {
        refPos_x.push_back(point(0));
        refPos_y.push_back(point(1));
    }

    // 初始参数
    double dt = 0.1;   // 时间步长
    double L = 2.9;    // 轴距
    double max_steer = 60 * PI / 180; // 最大方向盘转角
    double target_v = 30.0 / 3.6;     // 最大速度

    // 计算航向角和曲率
    std::vector<double> diff_x(refPos_x.size()), diff_y(refPos_y.size());
    for (size_t i = 0; i < refPos_x.size() - 1; ++i) {
        diff_x[i] = refPos_x[i+1] - refPos_x[i];
        diff_y[i] = refPos_y[i+1] - refPos_y[i];
    }
    diff_x.back() = diff_x[diff_x.size() - 2];
    diff_y.back() = diff_y[diff_y.size() - 2];

    std::vector<double> refHeading, refK, refDelta;
    for (size_t i = 0; i < refPos_x.size(); ++i) {
        double heading = std::atan2(diff_y[i], diff_x[i]);
        refHeading.push_back(heading);
        
        // 计算曲率
        double derivative1 = diff_y[i] / std::abs(diff_x[i]);
        double derivative2 = (diff_y[i+1] - diff_y[i]) / std::abs(diff_x[i]);  // 二阶导数
        double curvature = std::abs(derivative2) / std::pow(1 + derivative1 * derivative1, 1.5);
        refK.push_back(curvature);

        // 参考前轮转角
        refDelta.push_back(std::atan(L * curvature));
    }

    // 初始状态量
    double x = refPos_x[0] + 0.5;
    double y = refPos_y[0] + 0.5;
    double yaw = refHeading[0] + 0.02;
    double v = 0.1;
    Eigen::Vector2d U(0.01, 0.01);  // 初始控制量
    int idx = 0;
    std::vector<Eigen::Vector2d> pos_actual;
    std::vector<std::pair<int, double>> latError_MPC; // 保存误差

    // 循迹
    while (idx < refPos_x.size() - 1) {
        // 调用MPC控制器
        double Delta, latError;
        std::tie(Delta, v, idx, latError, U) = mpc_control(x, y, yaw, refPos_x, refPos_y, refHeading, refDelta, dt, L, U, target_v);

        // 误差太大，退出程序
        if (std::abs(latError) > 3) {
            std::cout << "误差过大，退出程序!" << std::endl;
            break;
        }

        // 更新状态量
        std::tie(x, y, yaw) = updateState(x, y, yaw, v, Delta, dt, L, max_steer);

        // 保存每一步的实际量
        pos_actual.emplace_back(x, y);
        latError_MPC.emplace_back(idx, latError);

        // 绘图或者显示位置，建议在终端或文件中记录每次位置
        std::cout << "当前坐标: (" << x << ", " << y << ")" << std::endl;
    }

    // 保存pos_actual和latError_MPC到文件中
    savePath(pos_actual, "path_MPC.txt");
    saveLatError(latError_MPC, "latError_MPC.txt");

    return 0;
}
