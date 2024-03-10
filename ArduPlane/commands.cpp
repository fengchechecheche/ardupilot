/*
 *  logic for dealing with the current command in the mission and home location
 */

#include "Plane.h"

/*
 *  set_next_WP - sets the target location the vehicle should fly to
 */
void Plane::set_next_WP(const struct Location &loc)
{
    if (auto_state.next_wp_crosstrack) {
        // copy the current WP into the OldWP slot
        prev_WP_loc = next_WP_loc;
        auto_state.crosstrack = true;
    } else {
        // we should not try to cross-track for this waypoint
        prev_WP_loc = current_loc;
        // use cross-track for the next waypoint
        auto_state.next_wp_crosstrack = true;
        auto_state.crosstrack = false;
    }

    // Load the next_WP slot
    // ---------------------
    next_WP_loc = loc;

    // if lat and lon is zero, then use current lat/lon
    // this allows a mission to contain a "loiter on the spot"
    // command
    if (next_WP_loc.lat == 0 && next_WP_loc.lng == 0) {
        next_WP_loc.lat = current_loc.lat;
        next_WP_loc.lng = current_loc.lng;
        // additionally treat zero altitude as current altitude
        if (next_WP_loc.alt == 0) {
            next_WP_loc.alt = current_loc.alt;
            next_WP_loc.relative_alt = false;
            next_WP_loc.terrain_alt = false;
        }
    }

    // convert relative alt to absolute alt
    if (next_WP_loc.relative_alt) {
        next_WP_loc.relative_alt = false;
        next_WP_loc.alt += home.alt;
    }

    // are we already past the waypoint? This happens when we jump
    // waypoints, and it can cause us to skip a waypoint. If we are
    // past the waypoint when we start on a leg, then use the current
    // location as the previous waypoint, to prevent immediately
    // considering the waypoint complete
    if (current_loc.past_interval_finish_line(prev_WP_loc, next_WP_loc)) {
        prev_WP_loc = current_loc;
    }

    // zero out our loiter vals to watch for missed waypoints
    loiter_angle_reset();

    setup_glide_slope();
    setup_turn_angle();
}

/*
 * 这段代码用于在引导模式下设置飞机的目标航点和其他相关参数。
 * 包括目标航点、高度、盘旋方向等，以确保飞机能够按照预期飞向指定的目标点。
 * */
void Plane::set_guided_WP(void)
{
    // 根据loiter_radius的值和guided_WP_loc的loiter_ccw标志来设置盘旋方向
    if (aparm.loiter_radius < 0 || guided_WP_loc.loiter_ccw) {
        // 逆时针盘旋
        loiter.direction = -1;
    } else {
        // 顺时针盘旋
        loiter.direction = 1;
    }

    // copy the current location into the OldWP slot
    // 将当前位置复制到prev_WP_loc，用于保存上一个航点的位置
    // ---------------------------------------
    prev_WP_loc = current_loc;

    // Load the next_WP slot
    // 将guided_WP_loc加载到next_WP_loc，表示下一个目标航点
    // ---------------------
    next_WP_loc = guided_WP_loc;

    // used to control FBW and limit the rate of climb
    // 根据当前位置或条件设置飞机的目标飞行高度。 
    // -----------------------------------------------
    set_target_altitude_current();

    // 通过下面两个方法，飞机被配置为以适当的滑翔坡度和转弯角度飞向目标点。
    // 设置滑翔坡度
    setup_glide_slope();
    // 设置转弯角度
    setup_turn_angle();

    // disable crosstrack, head directly to the point
    // 禁用crosstrack功能，直接飞向目标点
    auto_state.crosstrack = false;

    // reset loiter start time.
    // 重置盘旋开始时间，以便在之后计算盘旋的持续时间。
    loiter.start_time_ms = 0;

    // start in non-VTOL mode
    // 初始化为非垂直起降模式 
    auto_state.vtol_loiter = false;
    
    // 重置盘旋角度 
    loiter_angle_reset();

#if HAL_QUADPLANE_ENABLED
    // cancel pending takeoff
    // 如果启用了四轴飞机模式，则取消待定的起飞  
    quadplane.guided_takeoff = false;
#endif
}

/*
  update home location from GPS
  this is called as long as we have 3D lock and the arming switch is
  not pushed
*/
void Plane::update_home()
{
    if (hal.util->was_watchdog_armed()) {
        return;
    }
    if ((g2.home_reset_threshold == -1) ||
        ((g2.home_reset_threshold > 0) &&
         (fabsf(barometer.get_altitude()) > g2.home_reset_threshold))) {
        // don't auto-update if we have changed barometer altitude
        // significantly. This allows us to cope with slow baro drift
        // but not re-do home and the baro if we have changed height
        // significantly
        return;
    }
    if (ahrs.home_is_set() && !ahrs.home_is_locked()) {
        Location loc;
        if(ahrs.get_location(loc) && gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
            // we take the altitude directly from the GPS as we are
            // about to reset the baro calibration. We can't use AHRS
            // altitude or we can end up perpetuating a bias in
            // altitude, as AHRS alt depends on home alt, which means
            // we would have a circular dependency
            loc.alt = gps.location().alt;
            if (!AP::ahrs().set_home(loc)) {
                // silently fail
            }
        }
    }
    barometer.update_calibration();
    ahrs.resetHeightDatum();
}

bool Plane::set_home_persistently(const Location &loc)
{
    if (hal.util->was_watchdog_armed()) {
        return false;
    }
    if (!AP::ahrs().set_home(loc)) {
        return false;
    }

    // Save Home to EEPROM
    mission.write_home_to_storage();

    return true;
}
