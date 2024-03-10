#include "mode.h"
#include "Plane.h"

/*
 * 这段代码是在进入盘旋模式时调用的，用于初始化盘旋状态并执行一些必要的操作。
 *
 * */
bool ModeLoiter::_enter()
{
    // 告诉飞机在指定位置进行盘旋
    // 具体的盘旋参数（如盘旋半径、高度等）可能已经在之前设置好了，
    // 或者这个方法内部会使用默认值。
    plane.do_loiter_at_location();

    // 设置地形目标高度。
    // 这个方法可能会根据下一个航点（next_WP_loc）的位置和地形信息，
    // 计算出合适的目标高度，以确保飞机在盘旋时不会与地面或障碍物发生碰撞。
    plane.setup_terrain_target_alt(plane.next_WP_loc);

    // 重置盘旋角度
    // 这可能是为了确保每次进入盘旋模式时，飞机都从一个固定的角度或方向开始盘旋，
    // 从而保持飞行的一致性。
    plane.loiter_angle_reset();

    // 返回true，表示成功进入盘旋模式 
    return true;
}

/*
 * 这段代码在盘旋模式运行期间周期性调用的，
 * 用于更新飞机的导航控制参数，包括翻滚角、俯仰角和油门大小。
 * */
void ModeLoiter::update()
{
    // 该方法会基于当前飞机的位置、速度和航向，以及盘旋模式的参数，计算出合适的翻滚角。
    // 翻滚角决定了飞机在水平面上的旋转方向，用于调整飞机的航向。
    plane.calc_nav_roll();

    // 该方法会计算出合适的俯仰角。
    // 俯仰角决定了飞机在垂直面上的角度，用于控制飞机的升降，以确保飞机保持在设定的盘旋高度。
    plane.calc_nav_pitch();

    // 该方法会计算出合适的油门大小。
    // 油门大小决定了飞机的动力输出，用于调整飞机的速度和爬升/下降率，以维持盘旋状态。
    plane.calc_throttle();
}

/*
 * 这个方法用于判断当前飞机的航向是否与目标位置对齐。
 * 整体来看，isHeadingLinedUp方法用于在盘旋模式下判断飞机是否应该继续盘旋，
 * 还是已经准备好飞向下一个目标位置。它首先检查目标位置是否足够远，然后计算方位角，
 * 并最终根据方位角和当前航向来判断是否需要对齐。
 * 这种方法确保飞机在飞向目标时，航向是正确的，同时也防止在距离目标过近时继续无意义的盘旋。
 * 
 * 为什么会退出盘旋模式？
 * */
bool ModeLoiter::isHeadingLinedUp(const Location loiterCenterLoc, const Location targetLoc)
{
    // Return true if current heading is aligned to vector to targetLoc.
    // Tolerance is initially 10 degrees and grows at 10 degrees for each loiter circle completed.
    // 如果当前航向与指向目标位置的向量对齐，则返回true。  
    // 初始容差为10度，每完成一个盘旋圈，容差增加10度。

    // 获取盘旋半径 
    const uint16_t loiterRadius = abs(plane.aparm.loiter_radius);
    // 1.检查目标位置距离：
    // 检查目标位置targetLoc与盘旋中心loiterCenterLoc的距离是否小于盘旋半径加上半径的5%。
    // 如果是，那么维持盘旋状态会阻止飞机指向下一个目标位置，因此立即返回true，
    // 表示航向已经对齐，实际上这意味着飞机应该退出盘旋状态并飞向下一个目标。
    if (loiterCenterLoc.get_distance(targetLoc) < loiterRadius + loiterRadius*0.05) {
        /* Whenever next waypoint is within the loiter radius plus 5%,
           maintaining loiter would prevent us from ever pointing toward the next waypoint.
           Hence break out of loiter immediately
           如果目标位置与盘旋中心的距离小于盘旋半径加上半径的5%，
           则维持盘旋状态会阻止我们指向下一个目标位置。
           因此，立即退出盘旋状态
         */
        return true;
    }

    // 2.计算方位角：
    // 如果目标位置距离足够远，
    // 方法接着计算当前飞机位置plane.current_loc到目标位置targetLoc的方位角bearing_cd，
    // 这是一个以百分之一度为单位的角度值。
    // Bearing in centi-degrees
    // 获取当前位置到目标位置的方位角（以百分之一度为单位）
    const int32_t bearing_cd = plane.current_loc.get_bearing_to(targetLoc);

    // 3.判断航向是否对齐：
    // 将计算出的方位角作为参数传入，调用另一个方法来判断航向是否对齐 
    // 这个函数可能会根据飞机的当前航向和计算出的方位角，
    // 以及可能的容差范围（如注释中提到的初始容差为10度，并随盘旋圈数增加），
    // 来判断航向是否与目标位置对齐。
    return isHeadingLinedUp_cd(bearing_cd);
}


/*
 * 这个方法用于判断当前飞机的航向是否与给定的方位角bearing_cd对齐，
 * 并据此决定是否应该退出盘旋状态。
 * 同时，通过扩展容差，它提供了对强风等干扰因素的鲁棒性。
 * */
bool ModeLoiter::isHeadingLinedUp_cd(const int32_t bearing_cd)
{
    // Return true if current heading is aligned to bearing_cd.
    // Tolerance is initially 10 degrees and grows at 10 degrees for each loiter circle completed.
    // 如果当前航向与bearing_cd对齐，则返回true。  
    // 容差初始为10度，每完成一个盘旋圈，容差增加10度。

    // get current heading.
    // 获取当前航向，将其转换为百分之一度为单位  
    const int32_t heading_cd = (wrap_360(degrees(plane.ahrs.groundspeed_vector().angle())))*100;

    // 计算当前航向与给定方位角之间的差值，并处理为-180到180之间的值
    const int32_t heading_err_cd = wrap_180_cd(bearing_cd - heading_cd);

    /*
      Check to see if the the plane is heading toward the land
      waypoint. We use 20 degrees (+/-10 deg) of margin so that
      we can handle 200 degrees/second of yaw.
      检查飞机是否朝着着陆航点前进。我们使用20度（正负10度）的容差，  
      以便能够处理每秒200度的偏航。 

      After every full circle, extend acceptance criteria to ensure
      aircraft will not loop forever in case high winds are forcing
      it beyond 200 deg/sec when passing the desired exit course
      每完成一个完整的盘旋圈，就扩展接受准则，以确保飞机在遭遇迫使它  
      超出200度/秒的强风时不会无限循环地绕圈。 
    */

    // Use integer division to get discrete steps
    // 使用整数除法得到离散的步骤 
    // 方法根据飞机完成的盘旋圈数（通过plane.loiter.sum_cd表示）来计算扩展的容差expanded_acceptance。
    // 这允许飞机在遭遇强风或其他干扰时，能够更灵活地退出盘旋状态。
    const int32_t expanded_acceptance = 1000 * (labs(plane.loiter.sum_cd) / 36000);

    // 判断航向是否对齐
    // 如果航向误差小于等于1000（即10度）加上由于盘旋圈数增加而扩展的容差
    if (labs(heading_err_cd) <= 1000 + expanded_acceptance) {
        // Want to head in a straight line from _here_ to the next waypoint instead of center of loiter wp
        // 想要从当前位置直线飞向下一个航点，而不是盘旋中心的航点

        // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
        // 0表示从航点中心开始交叉追踪，1表示从切线出口位置开始交叉追踪
        if (plane.next_WP_loc.loiter_xtrack) {
            // 如果飞机的next_WP_loc.loiter_xtrack标志为true，
            // 方法会将下一个航点next_WP_loc更新为当前位置plane.current_loc。
            // 这可能是为了指示飞机应该直接从当前位置飞向下一个目标，
            // 而不是继续围绕盘旋中心的航点飞行。
            plane.next_WP_loc = plane.current_loc;
        }
        // 返回true，表示航向已经对齐。
        return true;
    }
    // 返回false，表示航向没有对齐。
    return false;
}

void ModeLoiter::navigate()
{
    // Zero indicates to use WP_LOITER_RAD
    plane.update_loiter(0);
}

