#include "kinematic_mpc.h"

void KinematicModel_MPC::updatestate(double accel, double delta_f){
    x += v * cos(yaw) * dt;
    y += v * sin(yaw) * dt;
    yaw += v * tan(delta_f) * dt;
    v += v + accel * dt;
}

std::tuple<double, double, double, double> KinematicModel_MPC::getstate(){
    return std::make_tuple(x, y, yaw, v);
}
