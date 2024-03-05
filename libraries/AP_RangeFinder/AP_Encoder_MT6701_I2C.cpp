#include "AP_Encoder_MT6701_I2C.h"

/*
 * 用APM的前后台架构，实现MT6701编码器的读取程序
 * AP_Encoder_MT6701_I2c.h 和 AP_Encoder_MT6701_I2c.cpp 负责“具体干活”
 * 驱动程序相关的API用C语言实现，在视频教程中没有被提到，后续再想办法
 */

AP_Encoder_MT6701_I2C::AP_Encoder_MT6701_I2C(AP_Encoder& encoder)
: AP_Encoder_Backend(encoder)
{

}

double
AP_Encoder_MT6701_I2C::read(void)
{

}
