#include "mode.h"
#include "Plane.h"

bool ModeGlide::_enter()
{
    gcs().send_text(MAV_SEVERITY_CRITICAL, "enter GLIDE mode.");

#if HAL_QUADPLANE_ENABLED
    // check if we should refuse auto mode due to a missing takeoff in
    // guided_wait_takeoff state
    /*
     * 当飞行器处于“guided_wait_takeoff”状态（即等待起飞指令的状态）时，
     * 系统会检查是否应该拒绝进入自动模式，原因可能是没有执行起飞动作。
     * 如果缺少起飞动作，系统可能会认为飞行器还没有准备好进入自动飞行模式，
     * 因此会拒绝或暂停进入自动模式。
     * 
     * 这段代码的目的是确保，在飞机进入guided模式并需要等待起飞时，
     * 如果任务不是以起飞命令开始，则发送错误消息并设置等待起飞的状态，
     * 防止飞机在没有起飞命令的情况下继续执行其他任务。
     */
    // 如果飞机之前处于guided模式，并且进入guided模式时等待起飞标志被设置
    if (plane.previous_mode == &plane.mode_guided &&
        quadplane.guided_wait_takeoff_on_mode_enter) {
        // 如果任务不是以起飞命令开始的  
        if (!plane.mission.starts_with_takeoff_cmd()) {
            // 通过地面控制站发送错误消息：“需要起飞航点” 
            gcs().send_text(MAV_SEVERITY_ERROR,"Takeoff waypoint required");
            // 设置等待起飞标志为true，这意味着系统现在处于等待起飞的状态。
            quadplane.guided_wait_takeoff = true;
            return false;
        }
    }
    
    /*
     * 这段代码与垂直起降（VTOL，Vertical Take-Off and Landing）功能有关
     * 这段代码的目的是根据quadplane的可用性和其特定的启用状态来动态地启用或禁用飞行器的VTOL模式。
     * VTOL模式通常允许飞行器垂直起飞和降落，这对于某些应用场景（如城市环境或有限空间）非常有用。
     */
    // 如果quadplane可用，并且quadplane的enable属性等于2  
    if (plane.quadplane.available() && plane.quadplane.enable == 2) {
        // 将飞行器的自动状态中的vtol_mode设置为true，表示启用VTOL模式  
        plane.auto_state.vtol_mode = true;
    } else {
        // 否则，将飞行器的自动状态中的vtol_mode设置为false，表示不启用VTOL模式 
        plane.auto_state.vtol_mode = false;
    }
#else
    plane.auto_state.vtol_mode = false;
#endif
    /*
     * 这行代码将plane对象的next_WP_loc（下一个航点位置）和prev_WP_loc（上一个航点位置）
     * 都设置为当前的位置（plane.current_loc）。
     * 这可能是在重置任务或者当飞机丢失当前任务上下文时的临时措施，
     * 确保飞机不会尝试飞向一个不存在的或错误的航点。
     */
    // 1.设置当前航点为下一个和上一个航点：
    plane.next_WP_loc = plane.prev_WP_loc = plane.current_loc;
    /*
     * 这行代码调用plane.mission对象的start_or_resume方法，
     * 根据任务的当前状态（可能是由MIS_AUTORESET参数控制的），开始或恢复飞行任务。
     */
    // start or resume the mission, based on MIS_AUTORESET
    // 2.开始或恢复任务：
    plane.mission.start_or_resume();

    /*
     * 看门狗（Watchdog）通常用于检测系统的故障或挂起，并在检测到问题时采取恢复措施。
     * 这里，代码首先检查看门狗是否被激活（was_watchdog_armed()）。
     * 如果看门狗被激活，代码进一步检查是否有一个特定的航点需要恢复：
     */
    // 3.处理看门狗（Watchdog）复位：
    if (hal.util->was_watchdog_armed()) {
        /*
         * hal.util->persistent_data.waypoint_num可能是一个持久化存储的变量，
         * 用于存储飞机在发生问题前应该前往的航点编号。
         * 如果它不等于0，说明飞机在之前因为某种原因（可能是看门狗触发）中断了任务，
         * 并且需要恢复到一个特定的航点。
         * 综上所述，这段代码主要是用于处理飞行任务中断后的恢复操作，
         * 确保飞机在遭遇问题后能够安全地继续或重新开始其任务。
         * 
         * 如果确实需要恢复到一个航点，代码执行以下操作：
         */
        if (hal.util->persistent_data.waypoint_num != 0) {
            // 通过地面控制站（GCS）发送一条信息，告知用户正在恢复到哪个航点。
            gcs().send_text(MAV_SEVERITY_INFO, "Watchdog: resume WP %u", hal.util->persistent_data.waypoint_num);
            // 设置当前命令为需要恢复的航点。
            plane.mission.set_current_cmd(hal.util->persistent_data.waypoint_num);
            // 将waypoint_num重置为0，表示航点恢复操作已经完成。
            hal.util->persistent_data.waypoint_num = 0;
        }
    }


#if HAL_SOARING_ENABLED
    /*
     * 如果HAL_SOARING_ENABLED被定义了，这行代码将被执行。
     * 它调用了plane对象的g2成员中的soaring_controller对象的init_cruising方法。
     * 这个方法的具体功能取决于它的实现，但从名字上看，
     * 它可能是用来初始化滑翔机的巡航状态的。
     */
    plane.g2.soaring_controller.init_cruising();
#endif

    return true;
}

/*
 * 这段代码是ModeAuto类中的_exit方法的实现，用于处理自动飞行模式退出时的逻辑。
 * 这段代码主要处理退出自动飞行模式时的任务停止和着陆序列重启逻辑。
 * 如果任务正在运行，它会被停止。
 * 如果当前导航命令是着陆命令（但不是垂直起降着陆命令），则重启着陆序列。
 * 最后，重置自动飞行模式的计时器。
 */
void ModeGlide::_exit()
{
    // 检查飞行任务是否正在运行  
    if (plane.mission.state() == AP_Mission::MISSION_RUNNING) {
        // 如果任务正在运行，则停止任务  
        plane.mission.stop();

        // 获取当前导航命令，并检查其ID是否为着陆命令（MAV_CMD_NAV_LAND）。
        // 如果是，则设置restart为true，表示需要重启着陆序列。
        bool restart = plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND;
// 如果启用了四轴飞行模式（HAL_QUADPLANE_ENABLED）  
#if HAL_QUADPLANE_ENABLED
        // 检查当前导航命令是否是垂直起降着陆命令  
        if (plane.quadplane.is_vtol_land(plane.mission.get_current_nav_cmd().id)) {
            // 如果当前导航命令是垂直起降着陆命令，则不重启着陆序列，将restart设置为false。
            restart = false;
        }
#endif
        // 如果需要重启着陆序列（即当前导航命令是着陆命令且不是垂直起降着陆）
        if (restart) {
            // 重启着陆序列  
            plane.landing.restart_landing_sequence();
        }
    }
    // 重置自动飞行模式开始飞行的计时器, 为下一次进入自动飞行模式时重新计时。
    plane.auto_state.started_flying_in_auto_ms = 0;
}

void ModeGlide::update()
{
    /*
     * 这段代码是检查飞行器的任务状态，并根据状态执行相应的操作。
     * 这段代码的主要目的是确保飞行器在自动模式下有一个正在运行的任务。
     * 如果没有，则自动将飞行器设置为返回起飞点模式，并通过GCS通知用户。
     * 这通常是一种安全措施，防止飞行器在没有任何任务的情况下继续执行不确定的行为。
     * */
    if (plane.mission.state() != AP_Mission::MISSION_RUNNING) {
        // this could happen if AP_Landing::restart_landing_sequence() returns false which would only happen if:
        // restart_landing_sequence() is called when not executing a NAV_LAND or there is no previous nav point
        // 如果飞行器的任务状态不是运行中（MISSION_RUNNING） 
        // 这可能发生在AP_Landing::restart_landing_sequence()返回false时，这只会发生在以下情况： 
        // 1. 当没有执行NAV_LAND导航点命令时调用了restart_landing_sequence()函数  
        // 2. 或者当没有前一个导航点时调用了restart_landing_sequence()函数 
        // 设置飞行器的模式为返回起飞点模式（RTL：Return To Launch）  
        // 设置模式切换的原因是因为任务结束（MISSION_END）  
        plane.set_mode(plane.mode_rtl, ModeReason::MISSION_END);
        // 通过地面控制系统（GCS）发送一条信息，告知飞行器当前在自动模式下但没有运行的任务
        gcs().send_text(MAV_SEVERITY_INFO, "Aircraft in auto without a running mission");
        // 结束当前函数或方法的执行  
        return;
    }

/*
 * 这段代码是条件编译和条件执行的结合。
 * 它涉及到的是四旋翼飞机（QuadPlane）的垂直起降（VTOL：Vertical Take-Off and Landing）模式下的自动控制。
 * 这段代码检查是否启用了四旋翼飞机的支持，并且如果当前四旋翼飞机处于VTOL自动模式，则调用其自动控制方法，
 * 并结束当前函数的执行。这通常是为了确保四旋翼飞机在特定模式下得到正确的控制处理，避免其他可能的控制逻辑干扰。
 * */
#if HAL_QUADPLANE_ENABLED
    // 如果HAL（硬件抽象层）中启用了四旋翼飞机（QuadPlane）的支持
    if (plane.quadplane.in_vtol_auto()) {
        // 如果四旋翼飞机当前处于VTOL自动模式
        // 调用四旋翼飞机的自动控制方法 
        plane.quadplane.control_auto();
        // 结束当前函数或方法的执行 
        return;
    }
#endif

    /*
     * 下面这段代码是关于飞行器的导航控制逻辑，具体地，
     * 它根据当前导航命令的ID (nav_cmd_id) 来决定如何计算飞行器的滚转、俯仰和油门控制。
     * 包括起飞、着陆、脚本导航和正常飞行模式。
     * 每种模式下都会计算相应的滚转、俯仰和油门控制值。
     * */
    // 1.获取当前导航命令ID:
    // 这行代码从飞行器的任务中获取当前导航命令的ID，并将其存储在nav_cmd_id变量中。
    uint16_t nav_cmd_id = plane.mission.get_current_nav_cmd().id;

    // 2.起飞逻辑：
    // 如果当前导航命令是起飞命令（MAV_CMD_NAV_TAKEOFF），
    // 或者当前是着陆命令但飞行阶段处于紧急中止着陆（FLIGHT_ABORT_LAND），
    // 则执行起飞时的滚转、俯仰和油门计算。
    if (nav_cmd_id == MAV_CMD_NAV_TAKEOFF ||
        (nav_cmd_id == MAV_CMD_NAV_LAND && plane.flight_stage == AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND)) {
        plane.takeoff_calc_roll();
        plane.takeoff_calc_pitch();
        plane.calc_throttle();
    } 
    // 3.着陆逻辑：
    // 如果当前导航命令是着陆命令（MAV_CMD_NAV_LAND），
    // 则执行着陆时的滚转和俯仰计算。
    else if (nav_cmd_id == MAV_CMD_NAV_LAND) {
        plane.calc_nav_roll();
        plane.calc_nav_pitch();

        // 【限制滚转】allow landing to restrict the roll limits
        // 这行代码将计算出的滚转值 (plane.nav_roll_cd) 通过landing.constrain_roll方法进行限制。
        // 它确保滚转值不会超过plane.g.level_roll_limit设定的限制值（乘以100转换为合适的单位）。
        // 这是为了保证着陆时飞行器的姿态稳定。
        plane.nav_roll_cd = plane.landing.constrain_roll(plane.nav_roll_cd, plane.g.level_roll_limit*100UL);

        // 【油门控制】检查着陆过程中油门是否被抑制。
        if (plane.landing.is_throttle_suppressed()) {
            // if landing is considered complete throttle is never allowed, regardless of landing type
            // 如果is_throttle_suppressed返回true，说明着陆已经完成或接近完成，此时不允许任何油门输入。
            // 将油门输出设置为0，即关闭油门。
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        } else {
            // 如果is_throttle_suppressed返回false，说明着陆还在进行中，需要计算油门值。
            // 调用calc_throttle方法计算油门控制。
            plane.calc_throttle();
        }
// 4.脚本导航逻辑 (仅当AP_SCRIPTING_ENABLED被定义时)
// 如果启用了脚本导航功能（AP_SCRIPTING_ENABLED），
// 并且当前导航命令是脚本时间命令（MAV_CMD_NAV_SCRIPT_TIME），
// 则直接使用传感器的滚转和俯仰值，并根据脚本设置来设置油门。
#if AP_SCRIPTING_ENABLED
    } else if (nav_cmd_id == MAV_CMD_NAV_SCRIPT_TIME) {
        // NAV_SCRIPTING has a desired roll and pitch rate and desired throttle
        plane.nav_roll_cd = plane.ahrs.roll_sensor;
        plane.nav_pitch_cd = plane.ahrs.pitch_sensor;
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.nav_scripting.throttle_pct);
#endif
    } 
    // 5.正常飞行逻辑:
    // 如果当前导航命令不是起飞、着陆或脚本导航，
    // 则执行正常自动飞行模式下的滚转、俯仰和油门计算。
    // 特殊情况下，如果导航命令不是MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT，
    // 则设置保持航向的值为-1。
    else {
        // we are doing normal AUTO flight, the special cases
        // are for takeoff and landing
        if (nav_cmd_id != MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT) {
            plane.steer_state.hold_course_cd = -1;
        }
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }
}

/*
 * 这段代码是ModeAuto类中的navigate方法的实现。这个方法主要用于在自动飞行模式下进行导航。
 * 在自动飞行模式下，飞行任务通常是一系列预设的导航命令，包括飞往不同的地点、执行不同的动作等。
 * 这个navigate方法的主要目的是在自动飞行模式下，根据预设的飞行任务进行导航。
 * 在调用这个方法之前，系统应该已经设置了“家”位置，并且已经加载了一个飞行任务。
 * 每次调用navigate方法时，它都会检查“家”位置是否设置，并更新飞行任务的状态，
 * 以便飞机能够按照预设的导航命令进行飞行。
 * */
void ModeGlide::navigate()
{
    // 检查是否已经设置了家（起飞点）位置
    if (AP::ahrs().home_is_set()) {
        // 如果已经设置，则更新飞行任务
        // update方法可能是用来检查当前应该执行哪个导航命令，并根据需要更新飞机的控制参数以执行该命令。
        plane.mission.update();
    }
}

/*
 * 这个函数的主要目的是确定当前是否在进行自动导航。
 * 如果启用了脚本导航功能（即定义了AP_SCRIPTING_ENABLED），
 * 则通过检查脚本导航是否活跃来确定是否在进行自动导航。
 * 如果没有启用脚本导航功能，则函数默认返回true，表示正在进行自动导航。
 * 这种设计可能是为了在不同的配置或功能集下提供一致的接口行为，
 * 即使某些特性（如脚本导航）没有被启用。
 * 
 * 这里的函数被标记为const，意味着这个函数不会修改类的任何成员变量。
 * */
bool ModeGlide::does_auto_navigation() const
{
#if AP_SCRIPTING_ENABLED
    /*
     * 这里调用了plane对象的nav_scripting_active方法，并取反其返回值。
     * 如果nav_scripting_active返回true（表示脚本导航是活跃的），
     * 则!plane.nav_scripting_active()会返回false，表示当前不在进行自动导航。
     * 反之，如果nav_scripting_active返回false，
     * 则!plane.nav_scripting_active()会返回true，表示当前在进行自动导航。
     * */
   return (!plane.nav_scripting_active());
#endif
    /*
     * 如果AP_SCRIPTING_ENABLED没有被定义，那么编译器将忽略之前的代码块，直接执行这一行代码。
     * 这意味着如果没有启用脚本导航功能，函数将始终返回true，表示当前正在进行自动导航。
     * */
   return true;
}

/*
 * 函数的主要目的是确定当前是否在使用自动油门控制。
 * 这个函数用于判断当前是否在使用自动油门控制。
 * 如果启用了脚本导航功能（即定义了AP_SCRIPTING_ENABLED），
 * 则通过检查脚本导航是否活跃来确定是否在使用自动油门控制。
 * 如果没有启用脚本导航功能，则函数默认返回true，表示正在使用自动油门控制。
 * 
 * 这里的函数被标记为const，意味着这个函数不会修改类的任何成员变量。
 * */
bool ModeGlide::does_auto_throttle() const
{
#if AP_SCRIPTING_ENABLED
    /*
     * 如果AP_SCRIPTING_ENABLED被定义，那么这行代码会被执行。
     * 这里调用了plane对象的nav_scripting_active方法，并取反其返回值。
     * 如果nav_scripting_active返回true（表示脚本导航是活跃的），
     * 则!plane.nav_scripting_active()会返回false，表示当前没有使用自动油门控制。
     * 反之，如果nav_scripting_active返回false，
     * 则!plane.nav_scripting_active()会返回true，表示当前在使用自动油门控制。
     * */
   return (!plane.nav_scripting_active());
#endif
    /*
     * 如果AP_SCRIPTING_ENABLED没有被定义，那么编译器将忽略之前的代码块，直接执行这一行代码。
     * 这意味着如果没有启用脚本导航功能，函数将始终返回true，表示当前正在使用自动油门控制。
     */
   return true;
}
