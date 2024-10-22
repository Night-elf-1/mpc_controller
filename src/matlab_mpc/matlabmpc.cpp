#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <quadprog.h>

using namespace Eigen;
using namespace std;

void mpc_control(double x, double y, double yaw, const VectorXd& refPos_x, const VectorXd& refPos_y,
                 const VectorXd& refPos_yaw, const VectorXd& refDelta, double dt, double L,
                 VectorXd& U, double target_v, double& Delta_real, double& v_real, int& idx, double& latError) {
    
    // MPC 预设参数
    const int Nx = 3;  // 状态量个数
    const int Nu = 2;  // 控制量个数
    const int Np = 60; // 预测步长
    const int Nc = 30; // 控制步长
    const double row = 10; // 松弛因子
    MatrixXd Q = 100 * MatrixXd::Identity(Np * Nx, Np * Nx); // (Np * Nx) x (Np * Nx)
    MatrixXd R = MatrixXd::Identity(Nc * Nu, Nc * Nu); // (Nc * Nu) x (Nc * Nu)

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
    latError = y_error * cos(heading_r) - x_error * sin(heading_r); // 横向误差

    // A, B 矩阵
    Matrix3d a;             // Ad矩阵
    a << 1, 0, -v_r * sin(heading_r) * dt,
         0, 1, v_r * cos(heading_r) * dt,
         0, 0, 1;

    MatrixXd b(3, 2);       // Bd矩阵
    b << cos(heading_r) * dt, 0,
         sin(heading_r) * dt, 0,
         tan(Delta_r) * dt / L, v_r * dt / (L * pow(cos(Delta_r), 2));

    // 状态空间方程的相关矩阵
    VectorXd kesi(Nx + Nu);             // 新状态变量kesi
    kesi.head(Nx) = X_real - Xr;        
    kesi.tail(Nu) = U;                  // U为初始控制量

    // A 矩阵
    MatrixXd A = MatrixXd::Zero(Nx + Nu, Nx + Nu);              // A矩阵为A3矩阵
    A.topLeftCorner(Nx, Nx) = a;
    A.topRightCorner(Nx, Nu) = b;
    A.bottomRightCorner(Nu, Nu) = MatrixXd::Identity(Nu, Nu);       // 往A3矩阵中添加值

    // B 矩阵
    MatrixXd B = MatrixXd::Zero(Nx + Nu, Nu);                       // B为B3矩阵
    B.topLeftCorner(Nx, Nu) = b;                                    // 往B3矩阵里面添加Bd矩阵和I矩阵(单位矩阵)
    B.bottomRightCorner(Nu, Nu) = MatrixXd::Identity(Nu, Nu);

    // C 矩阵
    MatrixXd C = MatrixXd::Zero(Nx, Nx + Nu);               // 设置C矩阵
    C.topLeftCorner(Nx, Nx) = MatrixXd::Identity(Nx, Nx);   // C=[E 0]

    // PHI 矩阵
    MatrixXd PHI = MatrixXd::Zero(Np * Nx, Nx + Nu);       // PHI为W矩阵 
    for (int i = 0; i < Np; ++i) {
        PHI.middleRows(i * Nx, Nx) = C * A.pow(i);
    }

    // THETA 矩阵
    MatrixXd THETA = MatrixXd::Zero(Np * Nx, Nc * Nu);          // THETA矩阵为Z矩阵
    for (int i = 0; i < Np; ++i) {
        for (int j = 0; j < Nc; ++j) {
            if (j <= i) {
                THETA.middleRows(i * Nx, Nx).middleCols(j * Nu, Nu) = C * A.pow(i - j) * B;
            }
        }
    }

    // H 矩阵
    MatrixXd H = THETA.transpose() * Q * THETA + R;
    VectorXd g = kesi.transpose() * PHI.transpose() * Q * THETA;

    // 约束
    MatrixXd A_I = MatrixXd::Zero(Nc * Nu, Nc * Nu);            // A_I对应Ae矩阵
    for (int i = 0; i < Nc; ++i) {
        A_I.block(i * Nu, 0, Nu, (i + 1) * Nu) = MatrixXd::Identity(Nu, Nu);
    }

    VectorXd Ut = VectorXd::Ones(Nc) * U;

    // 计算约束矩阵
    VectorXd Umax = VectorXd::Ones(Nc * Nu) * umax;
    VectorXd Umin = VectorXd::Ones(Nc * Nu) * umin;
    VectorXd delta_Umin = VectorXd::Ones(Nc * Nu) * delta_umin;
    VectorXd delta_Umax = VectorXd::Ones(Nc * Nu) * delta_umax;

    // 计算输出结果
    U(0) = kesi(3) + delta_v_tilde;
    U(1) = kesi(4) + delta_Delta_tilde;

    v_real = U(0) + v_r;
    Delta_real = U(1) + Delta_r;
}
