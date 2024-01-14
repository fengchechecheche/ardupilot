#include "Copter.h"

#if MODE_GUIDED_ENABLED == ENABLED

/*
 * Init and run calls for guided flight mode
 */



// init - initialise guided controller
bool ModeDrawStar::init(bool ignore_checks)
{
    path_num = 0;
    generate_path6_flap();

    pos_control_start();

    return true;
}

void ModeDrawStar::generate_path()
{
    float radius_cm = 1000.0;

    wp_nav->get_wp_stopping_point(path[0]);

    path[1] = path[0] + Vector3f(1.0f, 0, 0) * radius_cm;
    path[2] = path[0] + Vector3f(-cosf(radians(36.0f)), -sinf(radians(36.0f)), 0) * radius_cm;
    path[3] = path[0] + Vector3f(sinf(radians(18.0f)), cosf(radians(18.0f)), 0) * radius_cm;
    path[4] = path[0] + Vector3f(sinf(radians(18.0f)), -cosf(radians(18.0f)), 0) * radius_cm;
    path[5] = path[0] + Vector3f(-cosf(radians(36.0f)), sinf(radians(36.0f)), 0) * radius_cm;
    path[6] = path[1];
}


void ModeDrawStar::generate_path6()
{
    float radius_cm = 1000.0;

    wp_nav->get_wp_stopping_point(path[0]);

    path[1] = path[0] + Vector3f(0, 2 * cosf(radians(30.0f)), 0) * radius_cm;
    path[2] = path[0] + Vector3f(-cosf(radians(60.0f)), cosf(radians(30.0f)), 0) * radius_cm;
    path[3] = path[0] + Vector3f(-1 - cosf(radians(60.0f)), cosf(radians(30.0f)), 0) * radius_cm;
    path[4] = path[0] + Vector3f(-1, 0, 0) * radius_cm;
    path[5] = path[0] + Vector3f(-1 - cosf(radians(60.0f)), -cosf(radians(30.0f)), 0) * radius_cm;
    path[6] = path[0] + Vector3f(-cosf(radians(60.0f)), -cosf(radians(30.0f)), 0) * radius_cm;
    path[7] = path[0] + Vector3f(0, -2 * cosf(radians(30.0f)), 0) * radius_cm;
    path[8] = path[0] + Vector3f(cosf(radians(60.0f)), -cosf(radians(30.0f)), 0) * radius_cm;
    path[9] = path[0] + Vector3f(1 + cosf(radians(60.0f)), -cosf(radians(30.0f)), 0) * radius_cm;
    path[10] = path[0] + Vector3f(1, 0, 0) * radius_cm;
    path[11] = path[0] + Vector3f(1 + cosf(radians(60.0f)), cosf(radians(30.0f)), 0) * radius_cm;
    path[12] = path[0] + Vector3f(cosf(radians(60.0f)), cosf(radians(30.0f)), 0) * radius_cm;
    path[13] = path[0] + Vector3f(0, 2 * cosf(radians(30.0f)), 0) * radius_cm;
}


void ModeDrawStar::generate_path6_flip()
{
    float radius_cm = 1000.0;

    wp_nav->get_wp_stopping_point(path[0]);

    /*
     * 注：对于MATLAB中绘制和验证好的图形
     * 把 Y 轴纵坐标放在 Vector3f() 的第一个参数的位置
     * 把 X 轴横坐标放在 Vector3f() 的第二个参数的位置
     * 才能得到和MATLAB中方向一致的结果
     */
    path[1] = path[0] + Vector3f(2 * cosf(radians(30.0f)), 0, 0) * radius_cm;
    path[2] = path[0] + Vector3f(cosf(radians(30.0f)), -cosf(radians(60.0f)), 0) * radius_cm;
    path[3] = path[0] + Vector3f(cosf(radians(30.0f)), -1 - cosf(radians(60.0f)), 0) * radius_cm;
    path[4] = path[0] + Vector3f(0, -1, 0) * radius_cm;
    path[5] = path[0] + Vector3f(-cosf(radians(30.0f)), -1 - cosf(radians(60.0f)), 0) * radius_cm;
    path[6] = path[0] + Vector3f(-cosf(radians(30.0f)), -cosf(radians(60.0f)), 0) * radius_cm;
    path[7] = path[0] + Vector3f(-2 * cosf(radians(30.0f)), 0, 0) * radius_cm;
    path[8] = path[0] + Vector3f(-cosf(radians(30.0f)), cosf(radians(60.0f)), 0) * radius_cm;
    path[9] = path[0] + Vector3f(-cosf(radians(30.0f)), 1 + cosf(radians(60.0f)), 0) * radius_cm;
    path[10] = path[0] + Vector3f(0, 1, 0) * radius_cm;
    path[11] = path[0] + Vector3f(cosf(radians(30.0f)), 1 + cosf(radians(60.0f)), 0) * radius_cm;
    path[12] = path[0] + Vector3f(cosf(radians(30.0f)), cosf(radians(60.0f)), 0) * radius_cm;
    path[13] = path[0] + Vector3f(2 * cosf(radians(30.0f)), 0, 0) * radius_cm;
}


// run - runs the guided controller
// should be called at 100hz or more
void ModeDrawStar::run()
{
    if(path_num < 13)
    {
        if(wp_nav->reached_wp_destination())
        {
            path_num++;
            wp_nav->set_wp_destination(path[path_num], false);
        }
    }
    else if((path_num == 13) && wp_nav->reached_wp_destination())
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Draw star finished, now go into Loiter Mode.");
        copter.set_mode(Mode::Number::LOITER, ModeReason::MISSION_END);
    }

    pos_control_run();
 }



// initialise guided mode's position controller
void ModeDrawStar::pos_control_start()
{
    wp_nav->wp_and_spline_init();

    wp_nav->set_wp_destination(path[0], false);

    auto_yaw.set_mode_to_default(false);
}

// pos_control_run - runs the guided position controller
// called from guided_run
void ModeDrawStar::pos_control_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;

    if (!copter.failsafe.radio && use_pilot_yaw()) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // run position controllers
    // pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from position controller, yaw rate from pilot
        attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), target_yaw_rate);
    } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
        // roll & pitch from position controller, yaw rate from mavlink command or mission item
        attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), auto_yaw.rate_cds());
    } else {
        // roll & pitch from position controller, yaw heading from GCS or auto_heading()
        attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.yaw(), auto_yaw.rate_cds());
    }
}

#endif
