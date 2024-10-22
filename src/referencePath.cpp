#include "referencePath.h"

MyReferencePath::MyReferencePath(Eigen::Vector3d &inital_x, const double &end_x, const double &end_y){
    
}

refTraj MyReferencePath::CalRefTrajectory(std::vector<double> agv_state, parameters param, double dl=1.0){

}

std::vector<double> MyReferencePath::CalTrackError(Eigen::Vector3d inital_x){
    double x = inital_x(0), y = inital_x(1);
    double lat_error, yaw, k, min_index;
    return {lat_error, k, yaw, min_index};
}