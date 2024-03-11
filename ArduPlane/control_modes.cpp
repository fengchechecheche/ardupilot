#include "Plane.h"

#include "quadplane.h"
#include "qautotune.h"

Mode *Plane::mode_from_mode_num(const enum Mode::Number num)
{
    Mode *ret = nullptr;
    switch (num) {
    case Mode::Number::MANUAL:
        ret = &mode_manual;
        break;
    case Mode::Number::CIRCLE:
        ret = &mode_circle;
        break;
    case Mode::Number::STABILIZE:
        ret = &mode_stabilize;
        break;
    case Mode::Number::TRAINING:
        ret = &mode_training;
        break;
    case Mode::Number::ACRO:
        ret = &mode_acro;
        break;
    case Mode::Number::FLY_BY_WIRE_A:
        ret = &mode_fbwa;
        break;
    case Mode::Number::FLY_BY_WIRE_B:
        ret = &mode_fbwb;
        break;
    case Mode::Number::CRUISE:
        ret = &mode_cruise;
        break;
    case Mode::Number::AUTOTUNE:
        ret = &mode_autotune;
        break;
    case Mode::Number::AUTO:
        ret = &mode_auto;
        break;
    case Mode::Number::RTL:
        ret = &mode_rtl;
        break;
    case Mode::Number::LOITER:
        ret = &mode_loiter;
        break;
    case Mode::Number::AVOID_ADSB:
#if HAL_ADSB_ENABLED
        ret = &mode_avoidADSB;
        break;
#endif
    // if ADSB is not compiled in then fallthrough to guided
    case Mode::Number::GUIDED:
        ret = &mode_guided;
        break;
    case Mode::Number::INITIALISING:
        ret = &mode_initializing;
        break;
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QSTABILIZE:
        ret = &mode_qstabilize;
        break;
    case Mode::Number::QHOVER:
        ret = &mode_qhover;
        break;
    case Mode::Number::QLOITER:
        ret = &mode_qloiter;
        break;
    case Mode::Number::QLAND:
        ret = &mode_qland;
        break;
    case Mode::Number::QRTL:
        ret = &mode_qrtl;
        break;
    case Mode::Number::QACRO:
        ret = &mode_qacro;
        break;
#if QAUTOTUNE_ENABLED
    case Mode::Number::QAUTOTUNE:
        ret = &mode_qautotune;
        break;
#endif
#endif  // HAL_QUADPLANE_ENABLED
    case Mode::Number::TAKEOFF:
        ret = &mode_takeoff;
        break;
    case Mode::Number::THERMAL:
#if HAL_SOARING_ENABLED
        ret = &mode_thermal;
#endif
        break;
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::LOITER_ALT_QLAND:
        ret = &mode_loiter_qland;
        break;
#endif  // HAL_QUADPLANE_ENABLED

    }
    return ret;
}

// 这个函数的主要目的是读取控制开关的状态，并基于开关的位置更新飞行器的控制模式。
void Plane::read_control_switch()
{
    // 一个静态布尔变量，用于消除开关信号的抖动。
    static bool switch_debouncer;
    // 一个无符号8位整数变量，用于存储从 readSwitch 函数读取的开关位置。
    // 1.读取开关位置
    uint8_t switchPosition = readSwitch();

    // If switchPosition = 255 this indicates that the mode control channel input was out of range
    // If we get this value we do not want to change modes.
    // 2.检查开关位置是否有效
    // 如果 switchPosition 为 255，表示开关控制通道输入超出范围，函数直接返回，不执行任何操作。
    if(switchPosition == 255) return;

    // 3.检查是否有有效的遥控输入
    // 如果没有有效的遥控输入，函数直接返回。
    if (!rc().has_valid_input()) {
        // ignore the mode switch channel if there is no valid RC input
        return;
    }

    // 4.检查遥控输入是否过时
    // 如果当前的遥控输入信号比最近一次有效信号老超过0.1秒（100毫秒），则函数直接返回。
    if (millis() - failsafe.last_valid_rc_ms > 100) {
        // only use signals that are less than 0.1s old.
        return;
    }

    // 5.检查开关位置是否变化
    // 如果当前读取的开关位置与上一次的不同，则执行下面的代码块。
    // oldSwitchPosition：在代码中没有直接声明，但可以推测这是类的一个成员变量或全局变量，用于存储上一次读取的开关位置。
    if (oldSwitchPosition != switchPosition) {
        // 6.开关抖动消除
        // 如果 switch_debouncer 为 false，则将其设置为 true 并返回。
        // 这是为了确保只有当开关状态连续两次读取不同时，才进行模式切换，从而防止由于开关信号的短暂波动而导致的误操作。
        if (switch_debouncer == false) {
            // this ensures that mode switches only happen if the
            // switch changes for 2 reads. This prevents momentary
            // spikes in the mode control channel from causing a mode
            // switch
            switch_debouncer = true;
            return;
        }

        // 7.设置控制模式
        // 如果 switch_debouncer 为 true，则根据开关位置 switchPosition 设置飞行器的控制模式。
        // 这里假定 flight_modes 是一个数组，用于存储不同开关位置对应的控制模式。
        set_mode_by_number((enum Mode::Number)flight_modes[switchPosition].get(), ModeReason::RC_COMMAND);

        // 8.更新旧的开关位置
        // 更新 oldSwitchPosition 变量，使其存储当前读取的开关位置。
        oldSwitchPosition = switchPosition;
    }

    // 9.重置开关抖动消除器
    // 在函数的最后，将 switch_debouncer 重置为 false，以便在下一次读取开关位置时重新开始抖动消除过程。
    switch_debouncer = false;

}

// 【1】这个函数用于设置遥控器通道 8 的输入与飞机的飞行模式之间的关系。
// 它的功能是读取遥控器通道上的脉冲宽度，并根据脉冲宽度的范围来确定并返回控制开关的位置。
uint8_t Plane::readSwitch(void) const
{
    // 1.读取脉冲宽度
    // 调用 RC_Channels::get_radio_in 函数来读取遥控器指定通道的脉冲宽度，并将其存储在 pulsewidth 变量中。
    // 这里 g.flight_mode_channel - 1 是通道的索引，可能是因为数组或类似数据结构是从0开始索引，而配置可能是从1开始计数。
    // 在 config.h 中定义了FLIGHT_MODE_CHANNEL = 8
    uint16_t pulsewidth = RC_Channels::get_radio_in(g.flight_mode_channel - 1);

    // 2.检查错误条件
    // 如果脉冲宽度小于或等于900，或者大于或等于2200，函数返回255，表示这是一个错误条件。
    // 这可能是因为这些脉冲宽度范围超出了预期的开关位置范围。
    if (pulsewidth <= 900 || pulsewidth >= 2200) return 255;            // This is an error condition

    // 3.确定开关位置
    // 接下来的几个 if 语句根据脉冲宽度的范围来确定并返回开关的位置：
    if (pulsewidth <= 1230) return 0;
    if (pulsewidth <= 1360) return 1;
    if (pulsewidth <= 1490) return 2;
    if (pulsewidth <= 1620) return 3;
    if (pulsewidth <= 1749) return 4;              // Software Manual

    // 4.默认返回
    // 如果脉冲宽度大于1749（即没有满足前面的任何条件），函数返回5。
    // 这可能表示一个默认的开关位置，或者与某种硬件手册中定义的开关位置相对应。
    return 5;                                                           // Hardware Manual
}

void Plane::reset_control_switch()
{
    oldSwitchPosition = 254;
    read_control_switch();
}

/*
  called when entering autotune
 */
void Plane::autotune_start(void)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Started autotune");
    rollController.autotune_start();
    pitchController.autotune_start();
    yawController.autotune_start();
}

/*
  called when exiting autotune
 */
void Plane::autotune_restore(void)
{
    rollController.autotune_restore();
    pitchController.autotune_restore();
    yawController.autotune_restore();
    gcs().send_text(MAV_SEVERITY_INFO, "Stopped autotune");
}

/*
  enable/disable autotune for AUTO modes
 */
void Plane::autotune_enable(bool enable)
{
    if (enable) {
        autotune_start();
    } else {
        autotune_restore();
    }
}

/*
  are we flying inverted?
 */
bool Plane::fly_inverted(void)
{
    if (control_mode == &plane.mode_manual) {
        return false;
    }
    if (inverted_flight) {
        // controlled with aux switch
        return true;
    }
    if (control_mode == &mode_auto && auto_state.inverted_flight) {
        return true;
    }
    return false;
}
