#include "../../include/MPC_SPEED_STEEL/kinematic_mpc.h"

void KinematicModel_MPC::updatestate(double accel, double delta_f){
    // x += v * cos(yaw) * dt;
    // y += v * sin(yaw) * dt;
    // yaw += ((v * tan(delta_f)) / L) * dt;
    //v += accel * dt;

    x += accel * cos(yaw) * dt;
    y += accel * sin(yaw) * dt;
    yaw += ((accel * tan(delta_f)) / L) * dt;
}

std::tuple<double, double, double, double> KinematicModel_MPC::getstate(){
    return std::make_tuple(x, y, yaw, v);
}
