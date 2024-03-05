#include "AP_Encoder_Backend.h"

/*
 * 用APM的前后台架构，实现MT6701编码器的读取程序
 * AP_Encoder_Backend.h 和 AP_Encoder_Backend.cpp 负责后台类的实现
 */

AP_Encoder_Backend::AP_Encoder_Backend(AP_Encoder& encoder)
: _frontend(encoder)
{
    
}