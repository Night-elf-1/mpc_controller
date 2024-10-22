#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <OsqpEigen/OsqpEigen.h>  // 引入OSQP库

using namespace Eigen;
using namespace std;

// 定义MPC控制器函数
void mpc_control(double x, double y, double yaw, const VectorXd& refPos_x, const VectorXd& refPos_y,
                 const VectorXd& refPos_yaw, const VectorXd& refDelta, double dt, double L,
                 VectorXd& U, double target_v, double& Delta_real, double& v_real, int& idx, double& latError) {

    // MPC 预设参数
    const int Nx = 3;  // 状态量个数
    const int Nu = 2;  // 控制量个数
    const int Np = 60; // 预测步长
    const int Nc = 30; // 控制步长
    const double row = 10; // 松弛因子

    // 权重矩阵
    MatrixXd Q = 100 * MatrixXd::Identity(Np * Nx, Np * Nx);
    MatrixXd R = MatrixXd::Identity(Nc * Nu, Nc * Nu);

    // 控制量约束
    Vector2d umin(-0.2, -0.54);
    Vector2d umax(0.2, 0.332);
    Vector2d delta_umin(-0.05, -0.64);
    Vector2d delta_umax(0.05, 0.64);

    // 计算参考控制量
    idx = calc_target_index(x, y, refPos_x, refPos_y);
    double v_r = target_v;
    double Delta_r = refDelta(idx);
    double heading_r = refPos_yaw(idx);

    // 实际状态量与参考状态量
    Vector3d X_real(x, y, yaw);
    Vector3d Xr(refPos_x(idx), refPos_y(idx), refPos_yaw(idx));

    // 误差计算
    double x_error = x - refPos_x(idx);
    double y_error = y - refPos_y(idx);
    latError = y_error * cos(heading_r) - x_error * sin(heading_r);

    // A, B 矩阵
    Matrix3d a;
    a << 1, 0, -v_r * sin(heading_r) * dt,
         0, 1, v_r * cos(heading_r) * dt,
         0, 0, 1;

    MatrixXd b(3, 2);
    b << cos(heading_r) * dt, 0,
         sin(heading_r) * dt, 0,
         tan(Delta_r) * dt / L, v_r * dt / (L * pow(cos(Delta_r), 2));

    VectorXd kesi(Nx + Nu);
    kesi.head(Nx) = X_real - Xr;
    kesi.tail(Nu) = U;

    MatrixXd A = MatrixXd::Zero(Nx + Nu, Nx + Nu);
    A.topLeftCorner(Nx, Nx) = a;
    A.topRightCorner(Nx, Nu) = b;
    A.bottomRightCorner(Nu, Nu) = MatrixXd::Identity(Nu, Nu);

    MatrixXd B = MatrixXd::Zero(Nx + Nu, Nu);
    B.topLeftCorner(Nx, Nu) = b;
    B.bottomRightCorner(Nu, Nu) = MatrixXd::Identity(Nu, Nu);

    MatrixXd C = MatrixXd::Zero(Nx, Nx + Nu);
    C.block(0, 0, Nx, Nx) = MatrixXd::Identity(Nx, Nx);

    // **构建 PHI 和 THETA 矩阵**

    // PHI 矩阵（Nx * Nc, Nx + Nu），预测误差矩阵
    MatrixXd PHI = MatrixXd::Zero(Np * Nx, Nx + Nu);
    MatrixXd A_power = MatrixXd::Identity(Nx + Nu, Nx + Nu);  // A的幂次方，用于逐步迭代
    for (int i = 0; i < Np; ++i) {
        A_power = A_power * A;  // A 的 i 次幂
        PHI.block(i * Nx, 0, Nx, Nx + Nu) = C * A_power;  // PHI 的第 i 行
    }

    // THETA 矩阵（Nx * Nc, Nu * Nc），用于描述控制输入的作用
    MatrixXd THETA = MatrixXd::Zero(Np * Nx, Nc * Nu);
    A_power = MatrixXd::Identity(Nx + Nu, Nx + Nu);  // A的幂次方重置为单位矩阵

    for (int i = 0; i < Np; ++i) {
        MatrixXd B_sum = MatrixXd::Zero(Nx + Nu, Nu);
        for (int j = 0; j <= i && j < Nc; ++j) {
            B_sum += A_power * B;
            A_power = A_power * A;  // A 逐步累乘
            THETA.block(i * Nx, j * Nu, Nx, Nu) = C * B_sum;  // 填充 THETA 矩阵的第 (i, j) 块
        }
    }

    // 构建 H 矩阵和 g 向量
    MatrixXd H = THETA.transpose() * Q * THETA + R;
    VectorXd g = (kesi.transpose() * PHI.transpose() * Q * THETA).transpose();

    // **设置 lowerBound 和 upperBound**
    VectorXd lowerBound(2 * Nu * Nc);
    VectorXd upperBound(2 * Nu * Nc);

    for (int i = 0; i < Nc * Nu; ++i) {
        int controlIndex = i % Nu;    // 控制变量的索引（0 或 1）
        int timeIndex = i / Nu;       // 时间步的索引（0 到 Nc - 1）

        // **1. 控制增量 delta_u 的约束**
        lowerBound(i) = delta_umin(controlIndex);
        upperBound(i) = delta_umax(controlIndex);

        // **2. 控制输入 u 的累积约束**
        lowerBound(Nc * Nu + i) = umin(controlIndex) - U(controlIndex);
        upperBound(Nc * Nu + i) = umax(controlIndex) - U(controlIndex);
    }

    // **构建约束矩阵 A_t**
    Eigen::SparseMatrix<double> A_t(2 * Nc * Nu, Nc * Nu);

    // **1. 控制增量约束的矩阵部分（单位矩阵）**
    for (int i = 0; i < Nc * Nu; ++i) {
        A_t.insert(i, i) = 1.0;
    }

    // **2. 控制输入累积约束的矩阵部分（下三角矩阵）**
    for (int i = 0; i < Nc * Nu; ++i) {
        int controlIndex = i % Nu;
        int timeIndex = i / Nu;

        for (int j = 0; j <= timeIndex; ++j) {
            A_t.insert(Nc * Nu + i, j * Nu + controlIndex) = 1.0;
        }
    }

    // **OSQP求解器设置**
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    solver.data()->setNumberOfVariables(Nc * Nu);
    solver.data()->setNumberOfConstraints(2 * Nc * Nu);

    if (!solver.data()->setHessianMatrix(H.sparseView())) {
        cout << "Failed to set Hessian matrix" << endl;
        return;
    }

    if (!solver.data()->setGradient(g)) {
        cout << "Failed to set gradient vector" << endl;
        return;
    }

    if (!solver.data()->setLinearConstraintsMatrix(A_t)) {
        cout << "Failed to set constraint matrix" << endl;
        return;
    }

    if (!solver.data()->setLowerBound(lowerBound)) {
        cout << "Failed to set lower bound" << endl;
        return;
    }

    if (!solver.data()->setUpperBound(upperBound)) {
        cout << "Failed to set upper bound" << endl;
        return;
    }

    if (!solver.initSolver()) {
        cout << "Failed to initialize solver" << endl;
        return;
    }

    // **求解**
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        cout << "OSQP failed to solve the problem" << endl;
        return;
    }

    VectorXd QPSolution = solver.getSolution();

    // **提取控制增量 delta_u**
    VectorXd delta_U = QPSolution.head(Nu);

    // **更新控制量 U**
    U += delta_U;

    // **计算实际控制量**
    v_real = U(0) + v_r;
    Delta_real = U(1) + Delta_r;
}
