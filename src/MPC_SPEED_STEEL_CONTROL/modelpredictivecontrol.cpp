#include "../../include/MPC_SPEED_STEEL/modelpredictivecontrol.h"

vector<double> MPC_controller::calc_speed_profile(vector<double> rx, vector<double> ry, vector<double> ryaw, double target_speed){
    vector<double> speed_profile(rx.size(), target_speed);

    double direction = 1.0; //forward
    //Set stop point
    for( int i=0; i < rx.size()-1; i++){
        double dx = rx[i+1] - rx[i];
        double dy = ry[i+1] - ry[i];
        double move_direction = atan2(dy, dx);

        if (dx != 0.0 && dy != 0.0){
            double dangle = abs(YAW_P2P(move_direction - ryaw[i]));
            if (dangle >= M_PI/4.0) direction = -1.0;
            else direction = 1.0;
        }

        if (direction != 1.0) speed_profile[i] = -1 * target_speed;
        else speed_profile[i] = target_speed;

    }
    speed_profile[rx.size()-1] = 0.0;

    return speed_profile;
}

std::tuple<int, double> MPC_controller::calc_nearest_index(double current_x, double current_y, vector<double> cx, vector<double> cy, vector<double> cyaw, int pind){
    double mind = numeric_limits<double>::max();        // 初始化一个变量 mind，用于记录最小的距离平方值
    double ind = 0;                                     // 初始化索引 ind，用于存储找到的最近轨迹点的索引
    for(int i=pind; i<pind+N_IND_SEARCH; i++){          // N_IND_SEARCH 定义为10，开始遍历从索引 pind 开始的 N_IND_SEARCH 个轨迹点，逐个计算它们与小车当前状态的位置距离
        double idx = cx[i] - current_x;
        double idy = cy[i] - current_y;
        double d_e = std::sqrt(idx*idx + idy*idy);

        if (d_e<mind){
            mind = d_e;
            ind = i;
        }
    }

    return std::make_tuple(ind, mind);
}

void MPC_controller::smooth_yaw(vector<double>& cyaw){
    for(int i=0; i<cyaw.size()-1; i++){
        double dyaw = cyaw[i+1] - cyaw[i];

        while (dyaw > M_PI/2.0){
            cyaw[i+1] -= M_PI*2.0;
            dyaw = cyaw[i+1] - cyaw[i];
        }
        while (dyaw < -M_PI/2.0){
            cyaw[i+1] += M_PI*2.0;
            dyaw = cyaw[i+1] - cyaw[i];
        }
    }
}

std::tuple<int, double> MPC_controller::calc_ref_trajectory(double current_x, double current_y, vector<double> cx, vector<double> cy, vector<double> cyaw, int& target_ind){
    auto [ind, d_e] = calc_nearest_index(current_x, current_y, cx, cy, cyaw, target_ind);

    if (target_ind >= ind) ind = target_ind + 1;
    if (ind >= cx.size() - 1){
        ind = cx.size();
    }
    target_ind = ind;

    return std::make_tuple(target_ind, d_e);
}

std::tuple<double, double> MPC_controller::mpc_solve(vector<double>& cx, vector<double>& cy, vector<double>& cyaw, vector<double>& speed, vector<double>& ck, Eigen::Vector3d inital_x, int min_index, double min_errors, KinematicModel_MPC agv_model){
    const double row = 10;
    Eigen::Vector2d u_min(-0.01,-0.1570796);
    Eigen::Vector2d u_max(0,0.1570796);
    Eigen::Vector2d delta_umin(-0.05, -0.64);
    Eigen::Vector2d delta_umax(0.05, 0.64);

    double yaw_r = cyaw[min_index];
    double v_r = speed[min_index];
    double k_r = ck[min_index];
    double lat_error = min_errors;
    double delta_f_r = atan2(3.7 * k_r, 1);

    Eigen::Matrix3d Ad(3, 3);             // Ad矩阵
    Ad << 1, 0, -v_r * sin(yaw_r) * agv_model.dt,
         0, 1, v_r * cos(yaw_r) * agv_model.dt,
         0, 0, 1;

    Eigen::MatrixXd Bd(3, 2);       // Bd矩阵
    Bd << cos(yaw_r) * agv_model.dt, 0,
          sin(yaw_r) * agv_model.dt, 0,
          (tan(delta_f_r) / agv_model.L) * agv_model.dt, (v_r / (agv_model.L * std::pow(cos(delta_f_r), 2))) * agv_model.dt;

    // 状态空间方程的相关矩阵
    Eigen::VectorXd kesi(NX + NU);             // 新状态变量kesi  3 + 2
    Eigen::Vector3d x_r(cx[min_index], cy[min_index], yaw_r);
    kesi.head(NX) = inital_x - x_r;
    std::cout << "kesi(head) = " << kesi.head(NX) << std::endl;
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
        Eigen::MatrixXd result = A_3;
        //Eigen::MatrixXd result = Eigen::MatrixXd::Identity(NX + NU, NX + NU);
        for (int j = 0; j < i; ++j)
        {
            result = result * A_3;
        }
        
        W.middleRows(i * NX, NX) = C * result;
        //W.middleRows(i * NX, NX) = C * result.topLeftCorner(NX, NX + NU);
    }

    Eigen::MatrixXd Z = Eigen::MatrixXd::Zero(NP * NX, NC * NU);          // THETA矩阵为Z矩阵
    for (int i = 0; i < NP; ++i) {
        for (int j = 0; j < NC; ++j) {
            if (j <= i) {
                // 计算 A_3^(i-j)
                Eigen::MatrixXd result = Eigen::MatrixXd::Identity(A_3.rows(), A_3.cols()); // 初始化为单位矩阵
                for (int k = 0; k < (i - j); ++k) {
                    result *= A_3; // 逐步累乘A_3
                }

                Z.middleRows(i * NX, NX).middleCols(j * NU, NU) = C * result * B_3;
            }
        }
    }

    // H 矩阵
    Eigen::MatrixXd H = Z.transpose() * QB * Z + RB;
    Eigen::VectorXd g = kesi.transpose() * W.transpose() * QB * Z;

    // 约束
    Eigen::MatrixXd A_e = Eigen::MatrixXd::Zero(NC * NU, NC * NU);            // A_I对应Ae矩阵
    for (int i = 0; i < NC; ++i) {
        A_e.block(i * NU, 0, NU, (i + 1) * NU) = Eigen::MatrixXd::Identity(NU, NU*(i+1));
        //cout << 1 << endl;
    }

    Eigen::VectorXd U_t = Eigen::VectorXd::Zero(NC * NU);
    for (int i = 0; i < NC; ++i) {
        U_t.segment(i * NU, NU) = U; // 复制 U 到 U_t 的每个区段
    }

    Eigen::VectorXd Umax = Eigen::VectorXd::Ones(NC * NU);
    for (int i = 0; i < NC; ++i) {
        Umax.segment(i * NU, NU) = u_max;
    }

    Eigen::VectorXd Umin = Eigen::VectorXd::Ones(NC * NU);
    for (int i = 0; i < NC; ++i) {
        Umin.segment(i * NU, NU) = u_min;
    }

    Eigen::VectorXd delta_Umin = Eigen::VectorXd::Ones(NC * NU);
    for (int i = 0; i < NC; ++i) {
        delta_Umin.segment(i * NU, NU) = delta_umin;
    }

    Eigen::VectorXd delta_Umax = Eigen::VectorXd::Ones(NC * NU);
    for (int i = 0; i < NC; ++i) {
        delta_Umax.segment(i * NU, NU) = delta_umax;
    }

    OsqpEigen::Solver solver;
    int num_variables = H.rows();
    int num_constraints = A_e.rows();

    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(num_variables);
    solver.data()->setNumberOfConstraints(num_constraints);

    // 设置二次规划问题的矩阵和向量
    Eigen::SparseMatrix<double> H_sparse = H.sparseView();
    solver.data()->setHessianMatrix(H_sparse);  // H为稀疏矩阵
    solver.data()->setGradient(g);                   // 线性项 g

    // 设置约束的矩阵和边界
    Eigen::SparseMatrix<double> A_e_sparse = A_e.sparseView();
    solver.data()->setLinearConstraintsMatrix(A_e_sparse);  // A为稀疏矩阵
    solver.data()->setLowerBound(delta_Umin);                    // 下界
    solver.data()->setUpperBound(delta_Umax);                    // 上界

    // 初始化并求解问题
    if (!solver.initSolver()) {
        throw std::runtime_error("Solver initialization failed");
    }

    solver.solveProblem();

    // 获取求解结果
    Eigen::VectorXd solution = solver.getSolution();
    // 更新控制量U
    Eigen::VectorXd delta_U = solution.head(U.size());
    //std::cout << "delta_U = " << delta_U << std::endl;
    U += delta_U;
    //std::cout << "U = " << U << std::endl;

    // 计算实际的控制量
    double v_real = U(0) + v_r;
    //double v_real = U(0);
    double delta_real = U(1) + delta_f_r;
    //double delta_real = U(1);
    cout << "v_real = " << v_real
        << "delta_real = " << delta_real << endl;

    return std::make_tuple(v_real, delta_real);
}