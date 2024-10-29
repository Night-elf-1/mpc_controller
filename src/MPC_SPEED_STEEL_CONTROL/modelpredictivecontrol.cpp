#include "../../include/MPC_SPEED_STEEL/modelpredictivecontrol.h"

vector<double> MPC_controller::calc_speed_profile(vector<double> rx, vector<double> ry, vector<double> ryaw, double target_speed){
    vector<double> speed_profile(rx.size(), target_speed);

    double direction = 1.0; //forward
    //Set stop point
    for( int i=0; i < rx.size()-1; i++){
        double dx = rx[i+1] - rx[i];
        double dy = ry[i+1] - ry[i];
        double move_direction = atan2(dy, dx);

        if (dx != 0.0 && dy != 0.0){
            double dangle = abs(YAW_P2P(move_direction - ryaw[i]));
            if (dangle >= M_PI/4.0) direction = -1.0;
            else direction = 1.0;
        }

        if (direction != 1.0) speed_profile[i] = -1 * target_speed;
        else speed_profile[i] = target_speed;

    }
    speed_profile[rx.size()-1] = 0.0;

    return speed_profile;
}