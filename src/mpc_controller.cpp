#include "mpc_controller.h"

std::vector<double> shibo::controller::MPC_controller::calculate_linearMPC(Eigen::MatrixXd xref, Eigen::Vector3d inital_x, Eigen::MatrixXd dref, KinematicModel agv_model){
    int NX_mpc = xref.rows();                   // xref 和 dref初始化的矩阵大小为3行，4列和3列
    int NU_mpc = dref.rows();
    int NP_mpc = xref.cols() - 1;               // 3

    Eigen::MatrixXd x(NX_mpc, NP_mpc + 1);              // 创建状态变量矩阵x，3行，4列
    Eigen::MatrixXd u(NU_mpc, NP_mpc);                  // 创建控制矩阵，3行，3列
    std::vector<Eigen::MatrixXd> A_vec, B_vec;

    for (int i = 0; i < NP_mpc; ++i)
    {
        auto car_state_space = agv_model.statespace(dref(1, i), xref(2, i));    // 参数1：参考转角， 参数2：参考航向角
        A_vec.push_back(car_state_space[0]);
        B_vec.push_back(car_state_space[1]);
    }
    
    Eigen::VectorXd cost(NP_mpc + 1);                   // 创建一个一维动态大小数组
    std::vector<std::vector<int>> constrains;           // 创建二维的int类型数组

    for (int i = 0; i < NP_mpc; ++i)
    {
        cost(i) += (u.col(i) - dref.col(i)).transpose() * R * (u.col(i) - dref.col(i));          // 计算目标函数的控制累加部分
        if(i != 0) cost(i) += (x.col(i) - xref.col(i)).transpose() * Q * (x.col(i) - xref.col(i));

        Eigen::MatrixXd A = A_vec[i];                       // 取出第i时刻的A B矩阵
        Eigen::MatrixXd B = B_vec[i];

        constrains.push_back({(i + 1)*NX_mpc, (i + 1)*NX_mpc + NX_mpc});
        constrains.push_back( {i * NU_mpc, i * NU_mpc + NU_mpc} );

        x.col(i + 1) = A * x.col(i) + B * (u.col(i) - dref.col(i));
    }
    cost(NP_mpc) = (x.col(NP_mpc) - xref.col(i)).transpose() * Qd * (x.col(NP_mpc) - xref.col(NP_mpc));         // 终端代价

    x.col(0) = inital_x;                                    // 设置初始状态量
    
    Eigen::VectorXd lower_bound(NP_mpc * NU_mpc);           // 3 * 3
    Eigen::VectorXd upper_bound(NP_mpc * NU_mpc);

    for (int i = 0; i < NP_mpc; ++i)
    {
        lower_bound.segment(i * NU_mpc, NU_mpc) << -MAX_VEL, -MAX_STEER;            // 设置控制边界
        upper_bound.segment(i * NU_mpc, NU_mpc) << MAX_VEL, MAX_STEER;
    }
    
}