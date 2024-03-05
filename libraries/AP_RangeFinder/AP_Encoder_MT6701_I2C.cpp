#include "AP_Encoder_MT6701_I2C.h"
// #include "ssd1306.h"

/*
 * 用APM的前后台架构，实现MT6701编码器的读取程序
 * AP_Encoder_MT6701_I2c.h 和 AP_Encoder_MT6701_I2c.cpp 负责“具体干活”
 * 驱动程序相关的API用C语言实现，在视频教程中没有被提到，后续再想办法
 */

AP_Encoder_MT6701_I2C::AP_Encoder_MT6701_I2C(AP_Encoder& encoder)
: AP_Encoder_Backend(encoder)
{
    // 此处放硬件初始化的C代码
    // uint8_t check = SSD1306_Init();
    // while(check == 0){
    //     HAL_Delay(1);
    // }
    // SSD1306_Fill(SSD1306_COLOR_BLACK);
    // SSD1306_UpdateScreen();
}

double
AP_Encoder_MT6701_I2C::read(void)
{
    double fangle = 0;


    return fangle;
}
