#include "../../include/MPC_SPEED_STEEL/modelpredictivecontrol.h"
#include "../../include/matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main(int argc, char const *argv[])
{
    //生成参考路线
    vector<double> wx({10.0, 60.0, 125.0,  50.0,   60.0,  35.0,  -10.0});
    vector<double> wy({0.0,  0.0,  50.0,  65.0,   45.0,  50.0,  -20.0});

    Spline2D csp_obj(wx, wy);
    vector<double> r_x;
    vector<double> r_y;
    vector<double> ryaw;            // 航向角
    vector<double> rcurvature;      // 曲率
    vector<double> rs;
    for(double i=0; i<csp_obj.s.back(); i+=1.0){                                                // 计算出路径点X 和 Y
        vector<double> point_= csp_obj.calc_postion(i);
        r_x.push_back(point_[0]);
        r_y.push_back(point_[1]);
        ryaw.push_back(csp_obj.calc_yaw(i));
        rcurvature.push_back(csp_obj.calc_curvature(i));
        rs.push_back(i);
    }
    double target_speed = 0.75;
    
    parameters param;
    MPC_controller mpc(param.NX, param.NU, param.NP, param.NC);

    vector<double> speed_profile = mpc.calc_speed_profile(r_x, r_y, ryaw, target_speed);            // 计算出路径中的速度信息

    Eigen::Vector3d initial_x;                                        // 初始化agv初始状态 x y yaw
    initial_x << 10.0, 0.0, 0.0;

    KinematicModel_MPC agv(initial_x(0), initial_x(1), initial_x(2), 0.01, 3.7, 0.1);
    
    std::vector<double> x_history, y_history;
    std::cout << agv.x << " " << agv.y << std::endl;
    
    plt::figure_size(800, 600);      // 设置窗口大小
    plt::plot(r_x, r_y, "b--");      // 绘制参考路径
    // plt::scatter(std::vector<double>{initial_x(0)}, std::vector<double>{initial_x(1)}, 20.0, {{"color", "green"}});

    // mpc.calc_nearest_index(initial_x(0), initial_x(1), r_x, r_y, ryaw, target_ind);
    mpc.smooth_yaw(ryaw);

    while ( finish )
    {
        auto [min_index, min_e] = mpc.calc_ref_trajectory(initial_x(0), initial_x(1), r_x, r_y, ryaw, target_ind);

        auto [v_real, delta_real] = mpc.mpc_solve(r_x, r_y, ryaw, speed_profile, rcurvature, initial_x, min_index, min_e, agv);

        agv.updatestate(v_real, delta_real);

        auto [temp_x, temp_y, temp_yaw, temp_v] = agv.getstate();
        std::cout << "x = " << temp_x << "  y = " << temp_y << std::endl;
        initial_x << temp_x, temp_y, temp_yaw;
        x_history.push_back(temp_x);
        y_history.push_back(temp_y);

        plt::clf();                // 清除当前图像
        plt::plot(r_x, r_y, "b--");    // 绘制参考路径
        plt::plot(x_history, y_history, "r-");  // 绘制AGV的实际轨迹

        plt::scatter(std::vector<double>{initial_x(0)}, std::vector<double>{initial_x(1)}, 20.0, {{"color", "green"}});
        plt::pause(0.001);         // 暂停以更新图形

        if ( min_index >= r_x.size() - 15 )
        {
            finish = false;
        }
        
    }
    plt::show();  // 显示最终结果

    return 0;
}