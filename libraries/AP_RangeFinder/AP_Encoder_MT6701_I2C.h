#ifndef __AP_ENCODER_MT6701_I2C_H__
#define __AP_ENCODER_MT6701_I2C_H__

/*
 * 真正要通过IIC总线读取编码器数据的函数都在这个“具体干活”的类中进行实现
 * 这个“具体干活”的类是被统一的对象，它应该是派生于后台类的,是后台类的子类
 */

#include "AP_Encoder_Backend.h"

class AP_Encoder_MT6701_I2C : public AP_Encoder_Backend{
public:
    // 此时因为AP_Encoder_Backend.h中已经包含了AP_Encoder.h
    // 所以在引用AP_Encoder类时，就不用再重新包含一遍了
    AP_Encoder_MT6701_I2C(AP_Encoder& encoder);
    ~AP_Encoder_MT6701_I2C(){};    // 此处析构函数为空实现

    virtual double read(void);
};














#endif
