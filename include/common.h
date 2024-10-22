#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

//#define MAX_STEER 10 * (M_PI / 180)
#define MAX_ITER 3
#define DU_TH 0.1
#define MAX_TIME 5000
#define MAX_SPEED 0.9
#define MIN_SPEED 0.05
#define MAX_ACCEL 1.0

#define reartowheel 0.15
#define wheel_len 0.3
#define wheel_width 0.1

struct parameters{
    int L = 3.7;
    int NX = 3, NU = 2, NP = 60, NC = 30;
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

