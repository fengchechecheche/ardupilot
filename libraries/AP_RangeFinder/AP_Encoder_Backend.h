#ifndef __AP_ENCODER_BACKEND_H__
#define __AP_ENCODER_BACKEND_H__

/*
 * 后台类主要的作用是统一前台类和硬件设备的接口
 * 所以不会有太多的函数实现
 */


#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Encoder.h"

class AP_Encoder_Backend{
public:
    /* ---------------------------------------------- suwp方案代码 ------------------------------------------------------ */
    // 后台类一般会出现对前台类的引用
    // 也就是说在后台类的构造函数里，会出现一个对前台类引用
    // 这个时候就必须包含前台类的头文件
    // 此时通过添加前台类的引用，后台类就跟前台类建立起了联系
    // AP_Encoder_Backend(AP_Encoder& encoder);
    // ~AP_Encoder_Backend(){};    // 此处析构函数为空实现
    /* ---------------------------------------------- suwp方案代码 ------------------------------------------------------ */

    // constructor. This incorporates initialisation as well.
	AP_Encoder_Backend(AP_Encoder& encoder);

    // update the state structure
    virtual void update() = 0;
    virtual void init_serial(uint8_t serial_instance) {};

    // 此处定义read()函数不用执行具体的功能，只是用于统一接口
    // 因此把后台类中的read()函数定义为纯虚函数，要求子类中必须对这个函数进行实现
    virtual double read(void) = 0;
private:
    // 在后台类的私有成员中，会用到前台类
    // 在后台类中，如果想要通过对前台类的引用，调用到前台类和它的私有成员的话
    // 需要在前台类中声明后台类和后台类的子类为友元（friend class）类
    // 这样的话，后台类就可以通过_frontend调用到前台类中的私有成员
    AP_Encoder& _frontend;
};














#endif
