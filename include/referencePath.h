# include <iostream>
# include <vector>
# include <cmath>
# include <algorithm>
# include <eigen3/Eigen/Dense>
# include "common.h"

/**
 * @param xref 用于存储参考轨迹的x y坐标
 * @param dref 用于存储轨迹点的导数
 * @param ind 轨迹点索引
 */

struct refTraj{
    Eigen::MatrixXd xref, dref;
    int ind;
};

class MyReferencePath{
    public:
        MyReferencePath(Eigen::Vector3d &inital_x, const double &end_x, const double &end_y, std::vector<PathPoint> &trajectory);
        ~MyReferencePath();

        std::vector<double> CalTrackError(Eigen::Vector3d inital_x);

        // refTraj CalRefTrajectory(std::vector<double> agv_state, parameters param, double dl=1.0);

        std::vector<PathPoint> generateTrajectory(double x_start, double y_start, double yaw_start, double x_end, double y_end);

        double euclideanDistance(double x1, double y1, double x2, double y2);

        double calculateCurvature(double yaw_diff, double distance);
        
        double calculateVelocity(double curvature);

        std::pair<double, int> calcNearestIndexAndLateralError(double current_x, double current_y, 
                                                       const std::vector<PathPoint>& ref_path);
};