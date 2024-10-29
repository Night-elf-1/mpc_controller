#ifndef COMMON_H  // 如果没有定义 COMMON_H
#define COMMON_H  // 定义 COMMON_H
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

struct parameters{
    int L = 3.7;
    int NX = 3, NU = 2, NP = 50, NC = 10;
    double dt = 0.1, row = 10;
};

struct VehicleState
{
    double x;
    double y;
    double yaw;
    double kappa;
    double velocity;
    double angular_velocity;
    double acceleration;
};

struct PathPoint
{
    double x;
    double y;
    double yaw;
    double k;
    double v;
    double acceleration;
};

struct PathData
{
    vector<PathPoint> path_points;
};

struct LateralError
{
    double lateral_error;
    double heading_error;
};

struct ControlCommand
{
    double steer_angle;
    double acc;
};

#endif