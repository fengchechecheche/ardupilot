#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Common/Location.h>
#include <stdint.h>
#include <AP_Soaring/AP_Soaring.h>
#include <AP_ADSB/AP_ADSB.h>
#include <AP_Vehicle/ModeReason.h>
#include "quadplane.h"

class AC_PosControl;
class AC_AttitudeControl_Multi;
class AC_Loiter;
class Mode
{
public:

    /* Do not allow copies */
    Mode(const Mode &other) = delete;
    Mode &operator=(const Mode&) = delete;

    // Auto Pilot modes
    // ----------------
    enum Number : uint8_t {
        MANUAL        = 0,
        CIRCLE        = 1,
        STABILIZE     = 2,
        TRAINING      = 3,
        ACRO          = 4,
        FLY_BY_WIRE_A = 5,
        FLY_BY_WIRE_B = 6,
        CRUISE        = 7,
        AUTOTUNE      = 8,
        AUTO          = 10,
        RTL           = 11,
        LOITER        = 12,
        TAKEOFF       = 13,
        AVOID_ADSB    = 14,
        GUIDED        = 15,
        INITIALISING  = 16,
#if HAL_QUADPLANE_ENABLED
        QSTABILIZE    = 17,
        QHOVER        = 18,
        QLOITER       = 19,
        QLAND         = 20,
        QRTL          = 21,
#if QAUTOTUNE_ENABLED
        QAUTOTUNE     = 22,
#endif
        QACRO         = 23,
#endif
        THERMAL       = 24,
#if HAL_QUADPLANE_ENABLED
        LOITER_ALT_QLAND = 25,
#endif
        GLIDE         = 26,     // 新增滑翔模式
    };

    // Constructor
    Mode();

    // enter this mode, always returns true/success
    bool enter();

    // perform any cleanups required:
    void exit();

    // run controllers specific to this mode
    virtual void run() {};

    // returns a unique number specific to this mode
    virtual Number mode_number() const = 0;

    // returns full text name
    /*
     * 这两个函数都是 virtual，这意味着它们可以在派生类中被重写，而 = 0 表示它们是纯虚函数，
     * 这意味着任何从包含这两个函数的类派生出来的类都必须实现这两个函数。
     * 如果一个类包含纯虚函数，那么这个类就是抽象的，不能被实例化。
     * 
     * name() 函数返回一个指向 const char 的指针，这个指针指向一个表示飞行模式完整文本名称的字符串。
     * 由于它是 const，这表示通过这个函数返回的指针所指向的内容在函数执行期间不会被修改。
     * 
     * name4() 函数也返回一个指向 const char 的指针，但这个指针指向的字符串有一个特定的要求：
     * 它必须是恰好4个字节长。这通常用于返回一个简短的标识符或代码，可能用于在界面上显示、日志记录或通信协议中。
     * */
    virtual const char *name() const = 0;

    // returns a string for this flightmode, exactly 4 bytes
    virtual const char *name4() const = 0;

    // returns true if the vehicle can be armed in this mode
    virtual bool allows_arming() const { return true; }

    //
    // methods that sub classes should override to affect movement of the vehicle in this mode
    //

    // convert user input to targets, implement high level control for this mode
    virtual void update() = 0;

    // true for all q modes
    virtual bool is_vtol_mode() const { return false; }
    virtual bool is_vtol_man_throttle() const;
    virtual bool is_vtol_man_mode() const { return false; }

    // guided or adsb mode
    virtual bool is_guided_mode() const { return false; }

    // true if mode can have terrain following disabled by switch
    virtual bool allows_terrain_disable() const { return false; }

    // true if automatic switch to thermal mode is supported.
    virtual bool does_automatic_thermal_switch() const {return false; }

    // subclasses override this if they require navigation.
    virtual void navigate() { return; }

    // this allows certain flight modes to mix RC input with throttle
    // depending on airspeed_nudge_cm
    virtual bool allows_throttle_nudging() const { return false; }

    // true if the mode sets the vehicle destination, which controls
    // whether control input is ignored with STICK_MIXING=0
    virtual bool does_auto_navigation() const { return false; }

    // true if the mode sets the vehicle destination, which controls
    // whether control input is ignored with STICK_MIXING=0
    virtual bool does_auto_throttle() const { return false; }

    // method for mode specific target altitude profiles
    virtual bool update_target_altitude() { return false; }

    // handle a guided target request from GCS
    virtual bool handle_guided_request(Location target_loc) { return false; }

protected:

    // subclasses override this to perform checks before entering the mode
    virtual bool _enter() { return true; }

    // subclasses override this to perform any required cleanup when exiting the mode
    virtual void _exit() { return; }

#if HAL_QUADPLANE_ENABLED
    // References for convenience, used by QModes
    AC_PosControl*& pos_control;
    AC_AttitudeControl_Multi*& attitude_control;
    AC_Loiter*& loiter_nav;
    QuadPlane& quadplane;
    QuadPlane::PosControlState &poscontrol;
#endif
};


class ModeAcro : public Mode
{
public:

    Mode::Number mode_number() const override { return Mode::Number::ACRO; }
    const char *name() const override { return "ACRO"; }
    const char *name4() const override { return "ACRO"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeAuto : public Mode
{
public:

    Number mode_number() const override { return Number::AUTO; }
    const char *name() const override { return "AUTO"; }
    const char *name4() const override { return "AUTO"; }

    bool does_automatic_thermal_switch() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override;

    bool does_auto_throttle() const override;

protected:

    bool _enter() override;
    void _exit() override;
};

class ModeGlide : public Mode
{
public:
    Number mode_number() const override { return Number::GLIDE; }
    const char *name() const override { return "GLIDE"; }
    const char *name4() const override { return "GLID"; }

    bool does_automatic_thermal_switch() const override { return true; }
    // methods that affect movement of the vehicle in this mode
    void update() override;
    void navigate() override;
    bool allows_throttle_nudging() const override { return true; }
    bool does_auto_navigation() const override;
    bool does_auto_throttle() const override;
protected:
    bool _enter() override;
    void _exit() override;
};

class ModeAutoTune : public Mode
{
public:

    Number mode_number() const override { return Number::AUTOTUNE; }
    const char *name() const override { return "AUTOTUNE"; }
    const char *name4() const override { return "ATUN"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
    void _exit() override;
};

class ModeGuided : public Mode
{
public:

    Number mode_number() const override { return Number::GUIDED; }
    const char *name() const override { return "GUIDED"; }
    const char *name4() const override { return "GUID"; }

    // methods that affect movement of the vehicle in this mode
    // 这个函数可能用于更新模式相关的状态或数据。
    void update() override;

    // 用于导航飞行器
    void navigate() override;

    /*
     * 在C++中，virtual关键字用于声明一个函数为虚函数，这意味着它可以在派生类中被重写（override）。
     * override关键字则是一个C++11引入的特性，它用于显式地指示一个成员函数是重写基类中的虚函数。
     * 
     * 在你提供的代码段中：
     * is_guided_mode函数被声明为virtual，这意味着它可以在ModeGuided类的任何派生类中再次被重写。
     * 然而，在这个特定的例子中，is_guided_mode已经在ModeGuided类中实现了，并且返回true。
     * 这个函数还使用了const修饰符，这表示它不会修改类的任何成员变量（除非它们被声明为mutable），
     * 并且它可以在常量对象上被调用。
     * 
     * override关键字用于表明这个函数是重写基类Mode中的一个虚函数。使用override有以下几个好处：
     * 1.编译时检查：如果基类中没有名为is_guided_mode的虚函数，编译器将报错，
     *   这有助于防止由于拼写错误或忘记声明基类虚函数而导致的错误。
     * 2.代码可读性：override关键字清晰地告诉读者这个函数是重写的，这有助于理解类的层次结构和功能。
     * 
     * 在这个例子中，is_guided_mode函数简单地返回true，表明ModeGuided类确实代表了一个指导模式。
     * 这个函数的实现是简单直接的，并且不需要任何额外的逻辑或条件检查。
     * 需要注意的是，即使is_guided_mode函数在ModeGuided类中返回true，
     * 如果在ModeGuided的派生类中需要改变这个行为，那么派生类可以重写这个函数并返回不同的值。
     * 然而，在这个特定的实现中，由于ModeGuided类直接返回了true，如果没有特别的需求去改变这个行为，
     * 派生类通常不需要重写这个函数。
     * */
    virtual bool is_guided_mode() const override { return true; }

    // 表明在这个模式下允许微调油门
    /*
     * 在无人机、机器人或其他自动驾驶系统的上下文中，
     * 微调油门通常指的是允许操作员或控制系统在自动飞行或操作模式的基础上，进行微小的油门调整以微调行为。
     * 如果某个模式不允许这样的微调，那么这个函数就应该返回false。
     *
     * 在ModeGuided类中，allows_throttle_nudging函数返回true，表示在“指导”模式下，这种微调油门的操作是被允许的。
     * 如果其他模式不允许微调油门，那么它们应该重写这个函数并返回false。
     * 
     * 这个函数的实现非常直接，它只返回了一个硬编码的布尔值。
     * 在更复杂的实现中，这个函数可能会根据当前的状态或配置来动态地决定是否允许微调油门。
     * 然而，在这个例子中，它总是返回true，表明在“指导”模式下微调油门总是被允许的。
     * */
    
    /*
     * 声明重写的基类函数时，前一句代码带了“virtual”，后一句代码没有带“virtual”，两者没有任何区别。
     * 在C++中，当使用override关键字时，函数已经被声明为会重写基类中的虚函数，因此不需要显式地加上virtual关键字。
     * 编译器会自动理解这是一个虚函数的重写。所以，即使virtual关键字被省略，函数的行为仍然相同。
     * 
     * is_guided_mode函数显式地使用了virtual关键字，尽管这不是必须的，因为override已经表明了这一点。
     * 而allows_throttle_nudging函数没有显式地使用virtual关键字，但它仍然是一个虚函数的重写，因为override关键字的存在。
     * 
     * 从功能性和编译器的角度来看，这两个函数声明是等效的。在编译时，它们都会被处理为基类中虚函数的重写。
     * 在代码可读性方面，一些开发者可能更喜欢始终显式地使用virtual关键字，以便更清楚地表明函数的虚函数性质，尽管这不是必须的。
     * 
     * 总的来说，是否包含virtual关键字在override函数声明中是一个风格选择，而不是一个功能性的区别。
     * 在C++社区中，存在不同的编码风格和偏好，有些团队或项目可能更喜欢始终包含virtual关键字，而有些则可能省略它。
     * 关键是保持代码的一致性和可读性。
     * */
    bool allows_throttle_nudging() const override { return true; }

    // 表明这个模式支持自动导航
    bool does_auto_navigation() const override { return true; }

    // 表明这个模式支持自动油门控制
    bool does_auto_throttle() const override { return true; }

    // handle a guided target request from GCS
    // 处理来自地面控制系统（GCS）的指导目标请求。参数target_loc表示目标位置。
    bool handle_guided_request(Location target_loc) override;

protected:

    bool _enter() override;
};

class ModeCircle: public Mode
{
public:

    Number mode_number() const override { return Number::CIRCLE; }
    const char *name() const override { return "CIRCLE"; }
    const char *name4() const override { return "CIRC"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;
};

class ModeLoiter : public Mode
{
public:

    Number mode_number() const override { return Number::LOITER; }
    const char *name() const override { return "LOITER"; }
    const char *name4() const override { return "LOIT"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool isHeadingLinedUp(const Location loiterCenterLoc, const Location targetLoc);
    bool isHeadingLinedUp_cd(const int32_t bearing_cd);

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;
};

#if HAL_QUADPLANE_ENABLED
class ModeLoiterAltQLand : public ModeLoiter
{
public:

    Number mode_number() const override { return Number::LOITER_ALT_QLAND; }
    const char *name() const override { return "Loiter to QLAND"; }
    const char *name4() const override { return "L2QL"; }

    // handle a guided target request from GCS
    bool handle_guided_request(Location target_loc) override;

protected:
    bool _enter() override;

    void navigate() override;

private:
    void switch_qland();

};
#endif // HAL_QUADPLANE_ENABLED

class ModeManual : public Mode
{
public:

    Number mode_number() const override { return Number::MANUAL; }
    const char *name() const override { return "MANUAL"; }
    const char *name4() const override { return "MANU"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
};


class ModeRTL : public Mode
{
public:

    Number mode_number() const override { return Number::RTL; }
    const char *name() const override { return "RTL"; }
    const char *name4() const override { return "RTL "; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;

private:

    // Switch to QRTL if enabled and within radius
    bool switch_QRTL(bool check_loiter_target = true);
};

class ModeStabilize : public Mode
{
public:

    Number mode_number() const override { return Number::STABILIZE; }
    const char *name() const override { return "STABILIZE"; }
    const char *name4() const override { return "STAB"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
};

class ModeTraining : public Mode
{
public:

    Number mode_number() const override { return Number::TRAINING; }
    const char *name() const override { return "TRAINING"; }
    const char *name4() const override { return "TRAN"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
};

class ModeInitializing : public Mode
{
public:

    Number mode_number() const override { return Number::INITIALISING; }
    const char *name() const override { return "INITIALISING"; }
    const char *name4() const override { return "INIT"; }

    bool _enter() override { return false; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    bool allows_arming() const override { return false; }

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_throttle() const override { return true; }
};

class ModeFBWA : public Mode
{
public:

    Number mode_number() const override { return Number::FLY_BY_WIRE_A; }
    const char *name() const override { return "FLY_BY_WIRE_A"; }
    const char *name4() const override { return "FBWA"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

};

class ModeFBWB : public Mode
{
public:

    Number mode_number() const override { return Number::FLY_BY_WIRE_B; }
    const char *name() const override { return "FLY_BY_WIRE_B"; }
    const char *name4() const override { return "FBWB"; }

    bool allows_terrain_disable() const override { return true; }

    bool does_automatic_thermal_switch() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;
};

class ModeCruise : public Mode
{
public:

    Number mode_number() const override { return Number::CRUISE; }
    const char *name() const override { return "CRUISE"; }
    const char *name4() const override { return "CRUS"; }

    bool allows_terrain_disable() const override { return true; }

    bool does_automatic_thermal_switch() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool get_target_heading_cd(int32_t &target_heading) const;

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;

    bool locked_heading;
    int32_t locked_heading_cd;
    uint32_t lock_timer_ms;
};

#if HAL_ADSB_ENABLED
class ModeAvoidADSB : public Mode
{
public:

    Number mode_number() const override { return Number::AVOID_ADSB; }
    const char *name() const override { return "AVOID_ADSB"; }
    const char *name4() const override { return "AVOI"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    virtual bool is_guided_mode() const override { return true; }

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;
};
#endif

#if HAL_QUADPLANE_ENABLED
class ModeQStabilize : public Mode
{
public:

    Number mode_number() const override { return Number::QSTABILIZE; }
    const char *name() const override { return "QSTABILIZE"; }
    const char *name4() const override { return "QSTB"; }

    bool is_vtol_mode() const override { return true; }
    bool is_vtol_man_throttle() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }
    bool allows_throttle_nudging() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // used as a base class for all Q modes
    bool _enter() override;

    void run() override;

protected:
private:

    void set_tailsitter_roll_pitch(const float roll_input, const float pitch_input);
    void set_limited_roll_pitch(const float roll_input, const float pitch_input);

};

class ModeQHover : public Mode
{
public:

    Number mode_number() const override { return Number::QHOVER; }
    const char *name() const override { return "QHOVER"; }
    const char *name4() const override { return "QHOV"; }

    bool is_vtol_mode() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

protected:

    bool _enter() override;
};

class ModeQLoiter : public Mode
{
friend class QuadPlane;
friend class ModeQLand;
public:

    Number mode_number() const override { return Number::QLOITER; }
    const char *name() const override { return "QLOITER"; }
    const char *name4() const override { return "QLOT"; }

    bool is_vtol_mode() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

protected:

    bool _enter() override;
};

class ModeQLand : public Mode
{
public:

    Number mode_number() const override { return Number::QLAND; }
    const char *name() const override { return "QLAND"; }
    const char *name4() const override { return "QLND"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

    bool allows_arming() const override { return false; }

protected:

    bool _enter() override;
};

class ModeQRTL : public Mode
{
public:

    Number mode_number() const override { return Number::QRTL; }
    const char *name() const override { return "QRTL"; }
    const char *name4() const override { return "QRTL"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

    bool allows_arming() const override { return false; }

    bool does_auto_throttle() const override { return true; }

    bool update_target_altitude() override;

    bool allows_throttle_nudging() const override;

protected:

    bool _enter() override;
};

class ModeQAcro : public Mode
{
public:

    Number mode_number() const override { return Number::QACRO; }
    const char *name() const override { return "QACO"; }
    const char *name4() const override { return "QACRO"; }

    bool is_vtol_mode() const override { return true; }
    bool is_vtol_man_throttle() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

protected:

    bool _enter() override;
};

#if QAUTOTUNE_ENABLED
class ModeQAutotune : public Mode
{
public:

    Number mode_number() const override { return Number::QAUTOTUNE; }
    const char *name() const override { return "QAUTOTUNE"; }
    const char *name4() const override { return "QATN"; }

    bool is_vtol_mode() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    void run() override;

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
    void _exit() override;
};
#endif  // QAUTOTUNE_ENABLED

#endif  // HAL_QUADPLANE_ENABLED

class ModeTakeoff: public Mode
{
public:
    ModeTakeoff();

    Number mode_number() const override { return Number::TAKEOFF; }
    const char *name() const override { return "TAKEOFF"; }
    const char *name4() const override { return "TKOF"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

    // var_info for holding parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:
    AP_Int16 target_alt;
    AP_Int16 target_dist;
    AP_Int16 level_alt;
    AP_Int8 level_pitch;

    bool takeoff_started;
    Location start_loc;

    bool _enter() override;
};

#if HAL_SOARING_ENABLED

class ModeThermal: public Mode
{
public:

    Number mode_number() const override { return Number::THERMAL; }
    const char *name() const override { return "THERMAL"; }
    const char *name4() const override { return "THML"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // Update thermal tracking and exiting logic.
    void update_soaring();

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    // true if we are in an auto-throttle mode, which means
    // we need to run the speed/height controller
    bool does_auto_throttle() const override { return true; }

protected:

    bool exit_heading_aligned() const;
    void restore_mode(const char *reason, ModeReason modereason);

    bool _enter() override;
};

#endif
