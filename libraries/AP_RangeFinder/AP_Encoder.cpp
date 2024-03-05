#include "AP_Encoder.h"
#include "AP_Encoder_MT6701_I2C.h"

/*
 * 用APM的前后台架构，实现MT6701编码器的读取程序
 * AP_Encoder.h 和 AP_Encoder.cpp 负责前台类的实现
 */

AP_Encoder::AP_Encoder()
{

}

void 
AP_Encoder::init(void)
{
    // 此处就将前台类后后台“具体干活”的类通过后台类联系起来了
    _driver = new AP_Encoder_MT6701_I2C(*this);
}

double
AP_Encoder::read(void)
{
    double fangle = 0;

    fangle = _driver->read();

    return fangle;
}

