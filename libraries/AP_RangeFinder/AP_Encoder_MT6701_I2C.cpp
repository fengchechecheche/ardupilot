#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include "AP_Encoder_MT6701_I2C.h"

#define SEND_TEST_MESSAGE false
#define SAMPLE_FREQUENCY 0.01
#define MAX_LIMIT_factor 80
#define LPF_factor 200
#define Buff_Num 10

float angle_MT6701 = 0.0;
float break_angle_MT6701 = 0.0;
bool  break_angle_MT6701_flag = false;
float old_angle_MT6701 = 0.0;
float angle_MT6701_error = 0.0;
float relative_gear_rev = 0.0;
float new_relative_gear_rev = 0.0;
float old_relative_gear_rev = 0.0;
float avg_relative_gear_rev = 0.0;
float sum_relative_gear_rev = 0.0;
float relative_gear_rev_buff[Buff_Num] = {};
float gear_travel_angle = 0.0;
bool  gear_travel_angle_flag = false;

AP_Encoder_MT6701_I2C::AP_Encoder_MT6701_I2C(AP_Encoder &encoder, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_Encoder_Backend(encoder), _dev(std::move(dev)) {}

AP_Encoder_Backend *AP_Encoder_MT6701_I2C::detect(AP_Encoder &encoder, AP_HAL::OwnPtr<AP_HAL::I2CDevice> i2c_dev)
{
    if (!i2c_dev)
    {
        return nullptr;
    }
    AP_Encoder_MT6701_I2C *sensor = new AP_Encoder_MT6701_I2C(encoder, std::move(i2c_dev));
    if (!sensor)
    {
        return nullptr;
    }

    sensor->init();
    return sensor;
}

bool AP_Encoder_MT6701_I2C::init()
{
    gcs().send_text(MAV_SEVERITY_CRITICAL, "[2] run AP_Encoder_MT6701_I2C::init() start.\n");
    if (encoder_init())
    {
        return true;
    }
    return false;
}

bool AP_Encoder_MT6701_I2C::encoder_init()
{
    union
    {
        be16_t be16_val;
        uint8_t bytes[2];
    } timeout;

    // Retrieve lost signal timeout register
    const uint8_t read_reg1[2] = {SlaveAddress, ReadAddress1};
    const uint8_t read_reg2[2] = {SlaveAddress, ReadAddress2};

    if (((_dev->transfer(read_reg1, 2, timeout.bytes, 2)) && (_dev->transfer(read_reg2, 2, timeout.bytes, 2))) == true)
    {
        return false;
    }

    // call timer() at 100Hz.       10,000 us = 0.01 s
    // call timer() at 20Hz.        50,000 us = 0.05 s
    // call timer() at 2Hz.         500,000 us = 0.5 s
    // call timer() at 2s.          2,000,000 us = 2 s
    // call timer() at 2s.          5,000,000 us = 5 s
    _dev->register_periodic_callback(SAMPLE_FREQUENCY * 1000000, FUNCTOR_BIND_MEMBER(&AP_Encoder_MT6701_I2C::encoder_timer, void));

    return true;
}

void AP_Encoder_MT6701_I2C::encoder_timer(void)
{
    // 为 angle_f 赋初值，避免仿真编译报错
    float angle_f = 0.0;

    get_reading(angle_f);

    angle_MT6701 = angle_f;

    if ((break_angle_MT6701_flag == true))
    {
        break_angle_MT6701_flag = false;
        break_angle_MT6701 = angle_MT6701;
    }

    // 磁场角度没有从360度跨到0度的情况
    if (angle_MT6701 - old_angle_MT6701 > 0.0)
    {
        angle_MT6701_error = angle_MT6701 - old_angle_MT6701;

        if (angle_MT6701_error - MAX_LIMIT_factor < 0.0)
        {
            old_angle_MT6701 = angle_MT6701;
            // 对末端齿轮转速进行一阶低通滤波
            new_relative_gear_rev = angle_MT6701_error / 360.0 / SAMPLE_FREQUENCY;
            relative_gear_rev = ((255 - LPF_factor) * new_relative_gear_rev + LPF_factor * old_relative_gear_rev) / 255;
            old_relative_gear_rev = relative_gear_rev;

            // 存储齿轮转速平均值的变量 avg_relative_gear_rev 只有在齿轮转动的时候才会更新
            // 当限幅滤波不通过或齿轮停转转动时，变量 avg_relative_gear_rev 的值不会被更新。
            sum_relative_gear_rev = 0.0;
            for (uint8_t i = 0; i < Buff_Num - 1; i++)
            {
                relative_gear_rev_buff[i] = relative_gear_rev_buff[i + 1];
                sum_relative_gear_rev += relative_gear_rev_buff[i];
            }
            relative_gear_rev_buff[Buff_Num - 1] = relative_gear_rev;
            sum_relative_gear_rev += relative_gear_rev_buff[Buff_Num - 1];
            avg_relative_gear_rev = sum_relative_gear_rev / Buff_Num;
        }
        else
        {
            angle_MT6701_error = 0.0;
            relative_gear_rev = 0.0;
        }
    }
    // 磁场角度从360度跨到0度的情况
    else if (angle_MT6701 - old_angle_MT6701 < 0.0)
    {
        angle_MT6701_error = 360 - old_angle_MT6701 + angle_MT6701;

        // 如果磁场角度的偏差在20度以内，则更新齿轮转速
        // 否则考虑读取到的是异常数据，直接丢弃掉
        /*
         * 限幅滤波法
         * 1.方法限幅滤波法又称嵌位滤波法,或程序判断滤波法。这种滤波法的思路是:
         * 先根据经验判断,确定两次采样允许的最大偏差值(设为A)每次检测到新采样值时进行判断:
         * (1)如果本次新采样值与上次滤波结果之差<A,则本次采样值有效,令本次滤波结果=新采样值;
         * (2)如果本次采样值与上次滤波结果之差>A,则本次采样值无效,放弃本次值,令本次滤波结果=上次滤波结果。
         * 2.优点：能有效克服因偶然因素引起的脉冲干扰。
         * 3.缺点：无法抑制那种周期性的干扰,且平滑度差
         * */
        if (angle_MT6701_error - MAX_LIMIT_factor < 0.0)
        {
            old_angle_MT6701 = angle_MT6701;
            // 对末端齿轮转速进行一阶低通滤波
            new_relative_gear_rev = angle_MT6701_error / 360.0 / SAMPLE_FREQUENCY;
            relative_gear_rev = ((255 - LPF_factor) * new_relative_gear_rev + LPF_factor * old_relative_gear_rev) / 255;
            old_relative_gear_rev = relative_gear_rev;

            sum_relative_gear_rev = 0.0;
            for (uint8_t j = 0; j < Buff_Num - 1; j++)
            {
                relative_gear_rev_buff[j] = relative_gear_rev_buff[j + 1];
                sum_relative_gear_rev += relative_gear_rev_buff[j];
            }
            relative_gear_rev_buff[Buff_Num - 1] = relative_gear_rev;
            sum_relative_gear_rev += relative_gear_rev_buff[Buff_Num - 1];
            avg_relative_gear_rev = sum_relative_gear_rev / Buff_Num;
        }
        else
        {
            angle_MT6701_error = 0.0;
            relative_gear_rev = 0.0;
        }
    }
    else
    {
        angle_MT6701_error = 0.0;
        relative_gear_rev = 0.0;
    }
}

void AP_Encoder_MT6701_I2C::get_reading(float &reading_m)
{
    uint32_t angle = 0;
    float angle_f = 0;

    uint8_t ReadBuffer;
    const uint8_t read_reg3 = ReadAddress1;
    const uint8_t read_reg4 = ReadAddress2;
    // read the high and low byte distance registers
    if (_dev->transfer(&read_reg3, 1, &ReadBuffer, sizeof(ReadBuffer)))
    {
        angle = ReadBuffer;
        angle <<= 8;
    }
    if (_dev->transfer(&read_reg4, 1, &ReadBuffer, sizeof(ReadBuffer)))
    {
        angle += ReadBuffer;
        angle >>= 2;
        angle_f = (float)(angle * 360.0) / 16384.0;
        reading_m = angle_f;
    }
}

/*
   update the state of the sensor
*/
void AP_Encoder_MT6701_I2C::update(void)
{
    // nothing to do - its all done in the timer()
}
