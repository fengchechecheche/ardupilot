#include "mode.h"
#include "Plane.h"

/*
 * 这段代码用于在进入引导模式时初始化相关参数和状态。
 *
 * */
bool ModeGuided::_enter()
{
    // 关闭油门直通模式 
    plane.guided_throttle_passthru = false;
    /*
      when entering guided mode we set the target as the current
      location. This matches the behaviour of the copter code
      当进入引导模式时，我们将目标设置为当前位置。  
      这与直升机代码的行为相匹配。
    */
    // 设置引导模式下的目标航点为当前位置
    plane.guided_WP_loc = plane.current_loc;

// 如果启用了四轴飞机（Quadplane）模式 
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.guided_mode_enabled()) {
        /*
          if using Q_GUIDED_MODE then project forward by the stopping distance
          如果使用Q_GUIDED_MODE，则根据停止距离向前投影一个点作为目标
        */
        // 在当前航向上偏移一个点作为新的目标航点
        plane.guided_WP_loc.offset_bearing(
            // 获取地面速度向量的角度并转换为度
            degrees(plane.ahrs.groundspeed_vector().angle()),
            // 获取四轴飞机的停止距离  
            plane.quadplane.stopping_distance());
    }
#endif

    // 设置飞机的引导航点
    plane.set_guided_WP();
    // 返回true表示成功进入引导模式 
    return true;
}

/*
 * 这段代码用于在引导模式下更新飞机的导航参数和控制状态。
 *
 * */
void ModeGuided::update()
{
#if HAL_QUADPLANE_ENABLED
    // 如果飞机处于VTOL盘旋模式，并且四轴飞机可用，则调用四轴飞机的guided_update方法
    if (plane.auto_state.vtol_loiter && plane.quadplane.available()) {
        plane.quadplane.guided_update();
        // 更新完成后直接返回，不再执行后续代码
        return;
    }
#endif

    // 如果没有使用四轴飞机或者当前不处于VTOL盘旋模式，  
    // 则计算飞机的导航滚转角、导航俯仰角和油门值  
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}

/*
 * 这段代码用于在引导模式下更新飞机的盘旋参数。
 *
 * */
void ModeGuided::navigate()
{
    // Zero indicates to use WP_LOITER_RAD
    // 使用WP_LOITER_RAD作为盘旋半径  
    // 参数0指示使用默认值  
    plane.update_loiter(0);
}

/*
 * 这段代码作用是处理来自外部的请求，将飞机引导至指定的目标位置。
 *
 */
bool ModeGuided::handle_guided_request(Location target_loc)
{
    // 将传入的目标位置赋值给飞机的guided_WP_loc变量 
    plane.guided_WP_loc = target_loc;

    // add home alt if needed
    // 如果目标位置是相对高度
    if (plane.guided_WP_loc.relative_alt) {
        // 将目标位置的相对高度转换为绝对高度，即加上起飞点的海拔高度
        plane.guided_WP_loc.alt += plane.home.alt;
        // 标记目标位置的相对高度标志为0，表示现在使用的是绝对高度
        plane.guided_WP_loc.relative_alt = 0;
    }

    // 设置飞机的引导航点  
    plane.set_guided_WP();

    // 返回true，表示请求已成功处理  
    return true;
}
