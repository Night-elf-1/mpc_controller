#include "mpc_controller.h"

std::tuple<double, double> shibo::controller::MPC_controller::calculate_linearMPC_new(std::vector<PathPoint> &trajectory, Eigen::Vector3d inital_x, int min_index, double min_errors, KinematicModel agv_model){
    const double row = 10;
    Eigen::Vector2d u_min(-0.2, -0.54);
    Eigen::Vector2d u_max(0.2, 0.54);
    Eigen::Vector2d delta_umin(-0.05, -0.64);
    Eigen::Vector2d delta_umax(0.05, 0.64);

    double yaw_r = trajectory[min_index].yaw;
    double v_r = trajectory[min_index].v;
    double k_r = trajectory[min_index].k;
    double lat_error = min_errors;
    double delta_f_r = atan2(3.7 * k_r, 1);

    Eigen::Matrix3d Ad(3, 3);             // Ad矩阵
    Ad << 1, 0, -v_r * sin(yaw_r) * agv_model.mykinematic.dt,
         0, 1, v_r * cos(yaw_r) * agv_model.mykinematic.dt,
         0, 0, 1;
    // std::cout << "Ad: \n" << Ad << std::endl;

    Eigen::MatrixXd Bd(3, 2);       // Bd矩阵
    Bd << cos(yaw_r) * agv_model.mykinematic.dt, 0,
          sin(yaw_r) * agv_model.mykinematic.dt, 0,
          (tan(delta_f_r) * agv_model.mykinematic.dt / agv_model.mykinematic.L) , (v_r * agv_model.mykinematic.dt / (agv_model.mykinematic.L * std::pow(cos(delta_f_r), 2)));
    // std::cout << "Bd: \n" << Bd << std::endl;

    // 状态空间方程的相关矩阵
    Eigen::VectorXd kesi(NX + NU);             // 新状态变量kesi  3 + 2
    Eigen::Vector3d x_r(trajectory[min_index].x, trajectory[min_index].y, yaw_r);
    kesi.head(NX) = inital_x - x_r;        
    kesi.tail(NU) = U;                          // U为初始控制量
    // cout << "kesi = " << kesi << endl;

    Eigen::MatrixXd A_3 = Eigen::MatrixXd::Zero(NX + NU, NX + NU);              // A矩阵为A3矩阵
    A_3.topLeftCorner(NX, NX) = Ad;
    A_3.topRightCorner(NX, NU) = Bd;
    A_3.bottomRightCorner(NU, NU) = Eigen::MatrixXd::Identity(NU, NU);       // 往A3矩阵中添加值
    // std::cout << "A_3: \n" << A_3 << std::endl;

    Eigen::MatrixXd B_3 = Eigen::MatrixXd::Zero(NX + NU, NU);                       // B为B3矩阵
    B_3.topLeftCorner(NX, NU) = Bd;                                    // 往B3矩阵里面添加Bd矩阵和I矩阵(单位矩阵)
    B_3.bottomRightCorner(NU, NU) = Eigen::MatrixXd::Identity(NU, NU);
    // std::cout << "B_3: \n" << B_3 << std::endl;

    // C 矩阵
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(NX, NX + NU);               // 设置C矩阵
    C.topLeftCorner(NX, NX) = Eigen::MatrixXd::Identity(NX, NX);          // C=[E 0]
    // std::cout << "C: \n" << C << std::endl;

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
    // std::cout << "W: \n" << W << std::endl;

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
    // cout << " Z = " << Z << endl;

    // H 矩阵
    Eigen::MatrixXd H = Z.transpose() * QB * Z + RB;
    Eigen::VectorXd g = kesi.transpose() * W.transpose() * QB * Z;

    // 约束
    Eigen::MatrixXd A_e = Eigen::MatrixXd::Zero(NC * NU, NC * NU);            // A_I对应Ae矩阵
    for (int i = 0; i < NC; ++i) {
        A_e.block(i * NU, 0, NU, (i + 1) * NU) = Eigen::MatrixXd::Identity(NU, NU*(i+1));
        //cout << 1 << endl;
    }
    // cout << "A_e = " << A_e << endl;

    // Eigen::VectorXd U_t = Eigen::VectorXd::Ones(NC) * U;
    Eigen::VectorXd U_t = Eigen::VectorXd::Zero(NC * NU);
    for (int i = 0; i < NC; ++i) {
        U_t.segment(i * NU, NU) = U; // 复制 U 到 U_t 的每个区段
    }
    // cout << "u_t = " << U_t << endl;

    // 计算约束矩阵
    // Eigen::VectorXd Umax = Eigen::VectorXd::Ones(NC * NU) * u_max;
    Eigen::VectorXd Umax = Eigen::VectorXd::Ones(NC * NU);
    for (int i = 0; i < NC; ++i) {
        Umax.segment(i * NU, NU) = u_max;
    }

    // Eigen::VectorXd Umin = Eigen::VectorXd::Ones(NC * NU) * u_min;
    Eigen::VectorXd Umin = Eigen::VectorXd::Ones(NC * NU);
    for (int i = 0; i < NC; ++i) {
        Umin.segment(i * NU, NU) = u_min;
    }

    // Eigen::VectorXd delta_Umin = Eigen::VectorXd::Ones(NC * NU) * delta_umin;
    Eigen::VectorXd delta_Umin = Eigen::VectorXd::Ones(NC * NU);
    for (int i = 0; i < NC; ++i) {
        delta_Umin.segment(i * NU, NU) = delta_umin;
    }

    // Eigen::VectorXd delta_Umax = Eigen::VectorXd::Ones(NC * NU) * delta_umax;
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
    U += delta_U;

    // 计算实际的控制量
    double v_real = U(0) + v_r;
    double delta_real = U(1) + delta_f_r;

    return std::make_tuple(v_real, delta_real);
}