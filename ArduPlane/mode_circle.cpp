#include "mode.h"
#include "Plane.h"

/*
 * 这个函数通常在飞机进入圆形飞行模式时被调用，用于初始化或设置该模式所需的参数。
 * 这个_enter函数非常简洁，它只是简单地设置了圆形飞行的海拔高度，并返回了成功的标志。
 * 在实际应用中，根据系统的复杂性和需求，这个函数可能还包含其他初始化或设置步骤，
 * 比如设置圆形飞行的半径、中心位置、速度等参数。
 * */
bool ModeCircle::_enter()
{
    // the altitude to circle at is taken from the current altitude
    // 当前位置的海拔高度将用作圆形飞行的海拔高度
    // 这行代码将飞机的下一个目标位置（next_WP_loc）的海拔高度（alt）设置为当前位置的海拔高度（current_loc.alt）。
    // 这意味着飞机将在当前海拔高度上进行圆形飞行。
    // 在圆形飞行模式中，飞机通常会在一个恒定的海拔高度上绕圈，而不需要改变高度。
    plane.next_WP_loc.alt = plane.current_loc.alt;

    return true;
}

/*
 * 这个update方法负责更新飞机在“Circle”模式（圆形飞行模式）下的飞行参数。
 *
 * */
void ModeCircle::update()
{
    // we have no GPS installed and have lost radio contact
    // or we just want to fly around in a gentle circle w/o GPS,
    // holding altitude at the altitude we set when we
    // switched into the mode
    // 如果没有安装GPS且失去了无线电联系，  
    // 或者我们只是想在没有GPS的情况下以温和的方式绕圈飞行，  
    // 同时保持切换到该模式时设定的飞行高度。  
    
    // 1.设置滚转角
    // 设置飞机的滚转角为滚转限制角的1/3  
    // 飞机在圆形飞行模式时需要按照一个恒定的滚转角进行飞行，以维持圆形轨迹。
    // 这行代码将飞机的滚转角（nav_roll_cd）设置为飞机允许的最大滚转角（roll_limit_cd）的1/3。
    // 这确保了飞机在飞行时不会过于急剧地转弯，而是以一种相对温和的方式绕圈。
    plane.nav_roll_cd  = plane.roll_limit_cd / 3;

    // 2.更新载荷因子
    // 更新飞机的载荷因子，载荷因子（Load Factor）是飞机飞行时所受到的力与其重力的比值，
    // 它反映了飞机在飞行中的受力情况。更新载荷因子有助于飞机了解当前飞行状态，
    // 这对于飞行控制和稳定性至关重要。
    plane.update_load_factor();

    // 3.计算导航俯仰角
    // 计算飞机的导航俯仰角，俯仰角（Pitch）是飞机沿其纵轴旋转的角度，
    // 决定了飞机的上升或下降。计算导航俯仰角是飞行控制的一部分，
    // 它根据飞机的飞行路径、速度、目标高度等因素来确定飞机应该采取的俯仰角度，
    // 以实现特定的飞行目标。
    plane.calc_nav_pitch();

    // 4.计算油门输出 
    // 在圆形飞行模式中，油门输出需要根据飞机的速度、高度和所需飞行轨迹进行调整，
    // 以确保飞机能够维持稳定的飞行状态。
    plane.calc_throttle();
}

