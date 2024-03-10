#include "mode.h"
#include "Plane.h"

/*
 * 当飞机进入巡航模式时，这个方法会被调用以执行一些初始化或设置操作。
 *
 * */
bool ModeCruise::_enter()
{
    // 解锁航向，表示飞机可以自由改变航向，而不是保持在某个特定的航向上
    // locked_heading变量可能用于跟踪飞机是否锁定在当前航向上。
    // 在巡航模式下，飞机通常可以自由改变航向，因此这里将其设置为false。
    locked_heading = false;
    // 锁定计时器重置为0，表示目前没有锁定航向的计时操作  
    // lock_timer_ms变量可能用于跟踪航向锁定的时间（以毫秒为单位）。
    lock_timer_ms = 0;

#if HAL_SOARING_ENABLED
    // for ArduSoar soaring_controller
    // 如果启用了ArduSoar的滑翔上升控制，则初始化巡航模式  
    // 这可能是用于滑翔机或具有滑翔上升能力的飞机 
    plane.g2.soaring_controller.init_cruising();
#endif

    // 设置目标高度为当前高度，意味着飞机将在当前高度进行巡航 
    plane.set_target_altitude_current();

    // 返回true，表示成功进入巡航模式 
    return true;
}

/*
 * 这段代码负责在飞机处于巡航模式时更新飞行控制参数。
 * 总的来说，这个update方法负责在巡航模式下根据飞行员的输入和导航需求来计算和更新飞机的滚转角度以及其他相关飞行参数。
 * 如果飞行员没有操作副翼或方向舵，那么飞机可能会按照预设的导航路径飞行；
 * 如果飞行员进行了操作，那么飞机的航向将不再被锁定，飞行员将能够手动控制飞机的飞行方向。
 * */
void ModeCruise::update()
{
    /*
      in CRUISE mode we use the navigation code to control
      roll when heading is locked. Heading becomes unlocked on
      any aileron or rudder input
      在巡航模式下，我们使用导航代码来控制滚转，当航向被锁定时，  
      任何副翼或方向舵的输入都会使航向解锁。  
    */
    if (plane.channel_roll->get_control_in() != 0 || plane.channel_rudder->get_control_in() != 0) {
        // 如果副翼或方向舵有输入（即飞行员手动操作），则解锁航向
        locked_heading = false;
        // 航向锁定计时器重置为0 
        lock_timer_ms = 0;
    }

    // 判断航向是否锁定  
    if (!locked_heading) {
        // 如果航向未锁定，则根据副翼的输入来控制滚转 
        plane.nav_roll_cd = plane.channel_roll->norm_input() * plane.roll_limit_cd;
        // 更新载荷因子  
        plane.update_load_factor();
    } else {
        // 如果航向锁定，则使用导航代码来计算滚转
        plane.calc_nav_roll();
    }
    // 更新前馈后向速度和高度的控制  
    plane.update_fbwb_speed_height();
}

/*
 * 这段代码负责处理巡航模式下的导航逻辑。
 * 这个navigate方法负责在巡航模式下管理航向锁定功能，
 * 确保飞机在没有飞行员输入的情况下能够沿着GPS航向飞行，
 * 并在需要时更新导航点以维持飞行路径。
 * */
/*
  handle CRUISE mode, locking heading to GPS course when we have
  sufficient ground speed, and no aileron or rudder input
  处理巡航模式，当飞机有足够的地面速度，且没有副翼或方向舵输入时，  
  将航向锁定到GPS航向（即飞机起飞时机头的方向）。
 */
void ModeCruise::navigate()
{
    
    if (!locked_heading &&                              // 如果航向没有被锁定 
        plane.channel_roll->get_control_in() == 0 &&    // 且副翼没有输入 
        plane.rudder_input() == 0 &&                    // 且方向舵没有输入
        plane.gps.status() >= AP_GPS::GPS_OK_FIX_2D &&  // 且GPS状态至少为2D定位  
        plane.gps.ground_speed() >= 3 &&                // 且地面速度至少为3（可能是3米/秒或3节，取决于具体单位）
        lock_timer_ms == 0)                             // 且锁定计时器为0（即之前未开始计时）
    {
        // user wants to lock the heading - start the timer
        // 用户希望锁定航向 - 开始计时
        lock_timer_ms = millis();
    }
    if (lock_timer_ms != 0 &&               // 如果锁定计时器不为0  
        (millis() - lock_timer_ms) > 500)   // 且从计时开始到现在已经超过0.5秒  
    {
        // lock the heading after 0.5 seconds of zero heading input
        // from user
        // 锁定航向，因为用户在0.5秒内没有输入任何航向指令 
        locked_heading = true;
        // 重置锁定计时器
        lock_timer_ms = 0;
        // 记录当前GPS航向（以百分之一度为单位）
        locked_heading_cd = plane.gps.ground_course_cd();
        // 设置前一个航点位置为当前位置
        plane.prev_WP_loc = plane.current_loc;
    }
    // 如果航向被锁定
    if (locked_heading) {
        // 设置下一个航点位置为前一个航点位置
        plane.next_WP_loc = plane.prev_WP_loc;
        // always look 1km ahead
        // 总是向前看1公里
        // 根据锁定航向计算偏移，在当前位置到前一个航点位置的距离基础上加上1公里
        plane.next_WP_loc.offset_bearing(locked_heading_cd*0.01f, plane.prev_WP_loc.get_distance(plane.current_loc) + 1000);
        // 更新导航控制器的航点信息
        plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);
    }
}

/*
 * 这段代码用于获取当前锁定的目标航向（以百分之一度为单位）。
 * 这个函数的主要用途是允许外部代码查询当前巡航模式下是否锁定了航向，并获取锁定的目标航向值。
 * 例如，飞行控制系统的其他部分可能需要这个信息来执行进一步的导航或控制逻辑。
 * */
bool ModeCruise::get_target_heading_cd(int32_t &target_heading) const
{
    // 将目标航向的值赋给传入的引用参数target_heading
    target_heading = locked_heading_cd;
    // 返回锁定航向的状态（true表示已锁定，false表示未锁定） 
    return locked_heading;
}
