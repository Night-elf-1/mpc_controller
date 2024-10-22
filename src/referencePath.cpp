#include "referencePath.h"

MyReferencePath::MyReferencePath(Eigen::Vector3d &inital_x, const double &end_x, const double &end_y, std::vector<PathPoint> &trajectory){
    trajectory = generateTrajectory(inital_x(0), inital_x(1), inital_x(2), end_x, end_y);
}

refTraj MyReferencePath::CalRefTrajectory(std::vector<double> agv_state, parameters param, double dl=1.0){

}

std::vector<double> MyReferencePath::CalTrackError(Eigen::Vector3d inital_x){
    double x = inital_x(0), y = inital_x(1);
    double lat_error, yaw, k, min_index;
    return {lat_error, k, yaw, min_index};
}

double MyReferencePath::euclideanDistance(double x1, double y1, double x2, double y2){
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

double MyReferencePath::calculateCurvature(double yaw_diff, double distance){
    return yaw_diff / distance;  // 曲率的近似公式
}

double MyReferencePath::calculateVelocity(double curvature){
    double max_speed = 1.5;  // 最大速度
    double min_speed = 0.5;   // 最小速度
    double v = max_speed / (1 + std::abs(curvature));  // 简单的速度与曲率关系
    return std::max(v, min_speed);  // 速度不能低于最小值
}

std::vector<PathPoint> MyReferencePath::generateTrajectory(double x_start, double y_start, double yaw_start, double x_end, double y_end){
    std::vector<PathPoint> path_;
    // 计算起点和终点之间的欧氏距离
    double distance = euclideanDistance(x_start, y_start, x_end, y_end);

    // 根据距离决定轨迹点数，假设每米10个离散点
    int num_points = std::max(2, static_cast<int>(distance * 100));

    // 计算x和y的增量
    double dx = (x_end - x_start) / (num_points - 1);
    double dy = (y_end - y_start) / (num_points - 1);

    // 计算起点到终点的总航向角
    double yaw_end = std::atan2(dy, dx);

    // 生成轨迹
    for (int i = 0; i < num_points; ++i) {
        double x = x_start + i * dx;
        double y = y_start + i * dy;
        double yaw = yaw_start + i * (yaw_end - yaw_start) / (num_points - 1);  // 插值计算航向角

        // 计算曲率（假设简单直线路径）
        double curvature = calculateCurvature(yaw_end - yaw_start, distance);

        // 根据曲率计算速度
        double velocity = calculateVelocity(curvature);

        // 将生成的点加入轨迹
        path_.push_back({x, y, yaw, curvature, velocity});
    }

    return path_;
}