#include "referencePath.h"

MyReferencePath::MyReferencePath(Eigen::Vector3d &inital_x, const double &end_x, const double &end_y, std::vector<PathPoint> &trajectory){
    trajectory = generateTrajectory(inital_x(0), inital_x(1), inital_x(2), end_x, end_y);
}

MyReferencePath::~MyReferencePath(){}

// refTraj MyReferencePath::CalRefTrajectory(std::vector<double> agv_state, parameters param, double dl=1.0){

// }

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

std::pair<double, int> MyReferencePath::calcNearestIndexAndLateralError(double current_x, double current_y, const std::vector<PathPoint>& ref_path){
    int nearest_index = -1;
    double min_distance = std::numeric_limits<double>::max();
    double lateral_error = 0.0;

    for (size_t i = 0; i < ref_path.size(); ++i) {
        double dx = current_x - ref_path[i].x;
        double dy = current_y - ref_path[i].y;
        
        // 计算欧氏距离
        double distance = std::hypot(dx, dy);

        // 如果找到更近的点，更新最近点索引和最小距离
        if (distance < min_distance) {
            min_distance = distance;
            nearest_index = i;

            // 计算法向量的横向距离（假设 ref_path 提供航向角 yaw）
            double ref_yaw = ref_path[i].yaw;
            double normal_x = -std::sin(ref_yaw);  // 法向量的 X 方向分量
            double normal_y = std::cos(ref_yaw);   // 法向量的 Y 方向分量

            // 计算当前点到参考点的向量和法向量的点积（即横向误差）
            lateral_error = dx * normal_x + dy * normal_y;
        }
    }

    return {min_distance, nearest_index};
}