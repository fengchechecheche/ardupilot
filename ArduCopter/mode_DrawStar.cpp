#include "Copter.h"

/*
 * 该飞行模式更接近于引导模式
 * 引导模式的功能是在 Mission Planner 中点一个点，右键选择飞到此点
 * 飞控就自动引导无人机飞行到这个点，并悬停在这个点的位置
 * 当在别的地方再点一下时，无人机又飞到下一个点
 */

/*
 * 在五角星模式中，把五角星的顶点设置为目标航点
 * 飞控引导无人机飞到五角星的一个顶点后，返回已到达目标航点的消息
 * 此时再设置五角星的下一个顶点，以此类推
 */

#if MODE_DRAWSTAR_ENABLED == ENABLED

/*
 * 五角星航线模式初始化
 * 在 mode.cpp 的 set_mode() 方法中被调用
 * 当切换一个模式时，第一步一定是先初始化这个模式
 * 然后确定一个频率去调用它的run()方法
 */
bool ModeDrawStar::init(bool ignore_checks)
{
    path_num = 0;    // 把当前航点清零
    generate_path(); // 生成五角星航线

    pos_control_start(); // 开始位置控制

    return true;
}

// 生成五角星航线
void ModeDrawStar::generate_path()
{
    // 以五角星的外切圆定义航迹的大小
    float radius_cm = 1000.0;

    // 把停止点作为第 0 个航点
    wp_nav->get_wp_stopping_point(path[0]);

    // 具体计算以圆心为原点，五角星五个顶点的坐标
    // APM飞控中，以机头方向为 X 轴正向，以飞机的右侧为 Y 轴正向，飞机垂直于地面的下方为 Z 轴的正方向
    path[1] = path[0] + Vector3f(1.0f, 0, 0) * radius_cm;
    path[2] = path[0] + Vector3f(-cosf(radians(36.0f)), -sinf(radians(36.0f)), 0) * radius_cm;
    path[3] = path[0] + Vector3f(sinf(radians(18.0f)), cosf(radians(18.0f)), 0) * radius_cm;
    path[4] = path[0] + Vector3f(sinf(radians(18.0f)), -cosf(radians(18.0f)), 0) * radius_cm;
    path[5] = path[0] + Vector3f(-cosf(radians(36.0f)), sinf(radians(36.0f)), 0) * radius_cm;
    path[6] = path[1];
}

// 开始位置控制
void ModeDrawStar::pos_control_start()
{
    // initialise waypoint and spline controller
    // 航点导航库的初始化
    wp_nav->wp_and_spline_init();

    // no need to check return status because terrain data is not used
    // 把初始时刻的目标航点设置为path的第 0 点，相当于初始化
    wp_nav->set_wp_destination(path[0], false);

    // initialise yaw
    // 航点控制设置为默认
    auto_yaw.set_mode_to_default(false);
}

// 此模式的周期调用
// 在 mode.cpp 的 update_flight_mode() 中每隔2.5毫秒被调用一次
void ModeDrawStar::run()
{
    if (path_num < 6)
    { // 五角星航线尚未走完
        if (wp_nav->reached_wp_destination())
        { // 到达某个端点
            path_num++;
            wp_nav->set_wp_destination(path[path_num], false); // 将下一个航点位置设置为导航控制模块的目标位置
        }
    }
    else if((path_num == 6) && wp_nav->reached_wp_destination()){
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Draw star finished, now go into Loiter Mode.");
        copter.set_mode(Mode::Number::LOITER, ModeReason::MISSION_END);
    }

    // 任何情况下，不管航点是否更新，位置控制器都需要更新
    pos_control_run(); // 位置控制器
}

// guided_pos_control_run - runs the guided position controller
// called from guided_run
void ModeDrawStar::pos_control_run() // 注意，此函数直接从mode_guided.cpp中复制过来，不需要改其中的内容
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) //判断遥控器的航向输入
    {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate))
        {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // if not armed set throttle to zero and exit immediately
    // 如果没有准备好，把油门调到零，立即退出该模式
    if (is_disarmed_or_landed())
    {
        // 注：4.0.7中的代码在此处报错，仿照其他模式的代码进行了修改。
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    /* ----------------------------------------- 更新位置控制器-开始 ------------------------------------------ */
    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();
    /* ----------------------------------------- 更新位置控制器-结束 ------------------------------------------ */

    /* ----------------------------------------- 更新姿态控制器-开始 ------------------------------------------ */
    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD)
    {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    }
    else if (auto_yaw.mode() == AUTO_YAW_RATE)
    {
        // roll & pitch from waypoint controller, yaw rate from mavlink command or mission item
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.rate_cds());
    }
    else
    {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
    }
    /* ----------------------------------------- 更新姿态控制器-结束 ------------------------------------------ */
}

#endif