#include "mpc_controller.h"

std::vector<double> shibo::controller::MPC_controller::calculate_linearMPC(Eigen::MatrixXd xref, Eigen::Vector3d inital_x, Eigen::MatrixXd dref, KinematicModel agv_model){
    int NX_mpc = xref.rows();                   // xref 和 dref初始化的矩阵大小为3行，4列和3列
    int NU_mpc = dref.rows();
    int NP_mpc = xref.cols() - 1;

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

void shibo::controller::MPC_controller::calculate_linearMPC_new(Eigen::MatrixXd xref, Eigen::Vector3d inital_x, Eigen::MatrixXd dref, KinematicModel agv_model){
    const double row = 10;
    Eigen::Vector2d u_min(-0.2, -0.54);
    Eigen::Vector2d u_max(0.2, 0.332);
    Eigen::Vector2d delta_umin(-0.05, -0.64);
    Eigen::Vector2d delta_umax(0.05, 0.64);

    std::vector<double> track_error = MyReferencePath::CalTrackError(inital_x);
    double min_ind = track_error[3], yaw_r = track_error[2], lat_error = track_error[0];
    double v_r = dref[min_ind];
    double delta_f_r = dref[min_ind];           // 待修改

    Eigen::Matrix3d Ad;             // Ad矩阵
    Ad << 1, 0, -v_r * sin(yaw_r) * agv_model.mykinematic.dt,
         0, 1, v_r * cos(yaw_r) * agv_model.mykinematic.dt,
         0, 0, 1;

    Eigen::MatrixXd Bd(3, 2);       // Bd矩阵
    Bd << cos(yaw_r) * agv_model.mykinematic.dt, 0,
         sin(yaw_r) * agv_model.mykinematic.dt, 0,
         tan(delta_f_r) * agv_model.mykinematic.dt / agv_model.mykinematic.L, v_r * agv_model.mykinematic.dt / (agv_model.mykinematic.L * pow(cos(delta_f_r), 2));

    // 状态空间方程的相关矩阵
    Eigen::VectorXd kesi(NX + NU);             // 新状态变量kesi
    Eigen::Vector3d x_r();
    kesi.head(NX) = inital_x - x_r;        
    kesi.tail(NU) = U;                          // U为初始控制量

    Eigen::MatrixXd A_3 = Eigen::MatrixXd::Zero(NX + NU, NX + NU);              // A矩阵为A3矩阵
    A_3.topLeftCorner(NX, NX) = Ad;
    A_3.topRightCorner(NX, NU) = Bd;
    A_3.bottomRightCorner(NU, NU) = Eigen::MatrixXd::Identity(NU, NU);       // 往A3矩阵中添加值

    Eigen::MatrixXd B_3 = Eigen::MatrixXd::Zero(NX + NU, NU);                       // B为B3矩阵
    B_3.topLeftCorner(NX, NU) = Bd;                                    // 往B3矩阵里面添加Bd矩阵和I矩阵(单位矩阵)
    B_3.bottomRightCorner(NU, NU) = Eigen::MatrixXd::Identity(NU, NU);

    // C 矩阵
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(NX, NX + NU);               // 设置C矩阵
    C.topLeftCorner(NX, NX) = Eigen::MatrixXd::Identity(NX, NX);          // C=[E 0]

    Eigen::MatrixXd W = Eigen::MatrixXd::Zero(NP * NX, NX + NU);       // PHI为W矩阵 
    for (int i = 0; i < NP; ++i) {
        W.middleRows(i * NX, NX) = C * A_3.pow(i);
    }

    Eigen::MatrixXd Z = Eigen::MatrixXd::Zero(NP * NX, NC * NU);          // THETA矩阵为Z矩阵
    for (int i = 0; i < NP; ++i) {
        for (int j = 0; j < NC; ++j) {
            if (j <= i) {
                Z.middleRows(i * NX, NX).middleCols(j * NU, NU) = C * A_3.pow(i - j) * B_3;
            }
        }
    }

    // H 矩阵
    Eigen::MatrixXd H = Z.transpose() * QB * THETA + RB;
    Eigen::VectorXd g = kesi.transpose() * W.transpose() * QB * Z;

    // 约束
    Eigen::MatrixXd A_e = Eigen::MatrixXd::Zero(NC * NU, NC * NU);            // A_I对应Ae矩阵
    for (int i = 0; i < Nc; ++i) {
        A_e.block(i * NU, 0, NU, (i + 1) * NU) = Eigen::MatrixXd::Identity(NU, NU);
    }

    
}