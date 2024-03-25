#ifndef __AP_ENCODER_MT6701_I2C_H__
#define __AP_ENCODER_MT6701_I2C_H__

#include "AP_Encoder_Backend.h"
#include <AP_HAL/I2CDevice.h>

extern const AP_HAL::HAL &hal;
extern float angle_MT6701;
extern float break_angle_MT6701;
extern bool break_angle_MT6701_flag;
extern float angle_MT6701_error;
extern float relative_gear_rev;
extern float new_relative_gear_rev;
extern float old_relative_gear_rev;
extern float avg_relative_gear_rev;
extern float gear_travel_angle;
#define SlaveAddress 0X06 //MT6701 地址
#define ReadAddress1 0X03 // 数据高位寄存器地址
#define ReadAddress2 0X04 // 数据低位寄存器地址

class AP_Encoder_MT6701_I2C : public AP_Encoder_Backend{
public:
    // 此时因为AP_Encoder_Backend.h中已经包含了AP_Encoder.h
    // 所以在引用AP_Encoder类时，就不用再重新包含一遍了
    AP_Encoder_MT6701_I2C(AP_Encoder& encoder);
    ~AP_Encoder_MT6701_I2C(){};    // 此处析构函数为空实现

    // static detection function
    static AP_Encoder_Backend *detect(AP_Encoder& encoder, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // update state
    void update(void) override;
private:
    bool encoder_init();
    void encoder_timer(void);
    void get_reading(float &reading_m);
    // constructor
    AP_Encoder_MT6701_I2C(AP_Encoder& encoder, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);
    bool init();
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};

#endif
