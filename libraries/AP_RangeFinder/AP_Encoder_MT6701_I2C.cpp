#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include "AP_Encoder_MT6701_I2C.h"

float angle_MT6701 = 0.0;
float old_angle_MT6701 = 0.0;
float relative_gear_rev = 0.0;
#define SEND_TEST_MESSAGE false

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
        hal.console->printf("Found MT6701 Encoder.\n");
        return true;
    }
    hal.console->printf("Encoder not found.\n");
    return false;
}

bool AP_Encoder_MT6701_I2C::encoder_init()
{
    hal.scheduler->delay(10);
    gcs().send_text(MAV_SEVERITY_CRITICAL, "[3-1] run AP_Encoder_MT6701_I2C::encoder_init() start.\n");
    hal.scheduler->delay(10);
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
        hal.scheduler->delay(10);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "[3-2] run AP_Encoder_MT6701_I2C::encoder_init() failed.\n");
        gcs().send_text(MAV_SEVERITY_CRITICAL, "[3-2-1] timeout.bytes[0]: %d.\n", timeout.bytes[0]);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "[3-2-2] timeout.bytes[1]: %d.\n", timeout.bytes[1]);
        hal.scheduler->delay(10);
        return false;
    }
    hal.scheduler->delay(10);
    gcs().send_text(MAV_SEVERITY_CRITICAL, "[3-3-1] timeout.bytes[0]: %d.\n", timeout.bytes[0]);
    gcs().send_text(MAV_SEVERITY_CRITICAL, "[3-3-2] timeout.bytes[1]: %d.\n", timeout.bytes[1]);
    hal.scheduler->delay(10);

    // call timer() at 100Hz.       10,000 us = 0.01 s 
    // call timer() at 20Hz.        50,000 us = 0.05 s 
    // call timer() at 2Hz.         500,000 us = 0.5 s 
    // call timer() at 2s.          2,000,000 us = 2 s 
    // call timer() at 2s.          5,000,000 us = 5 s 
    _dev->register_periodic_callback(50000, FUNCTOR_BIND_MEMBER(&AP_Encoder_MT6701_I2C::encoder_timer, void));
    hal.scheduler->delay(10);
    gcs().send_text(MAV_SEVERITY_CRITICAL, "[3-4] run AP_Encoder_MT6701_I2C::encoder_init() success.\n");
    hal.scheduler->delay(10);

    return true;
}

void AP_Encoder_MT6701_I2C::encoder_timer(void)
{  
    // 为 angle_f 赋初值，避免仿真编译报错
    float angle_f = 0.0;

    get_reading(angle_f);

    angle_MT6701 = angle_f;

    if(angle_MT6701 - old_angle_MT6701 > 0.0)
    {
        relative_gear_rev = (angle_MT6701 - old_angle_MT6701) / 360.0 / 0.05 ;
        old_angle_MT6701 = angle_MT6701;
    }
    else if(angle_MT6701 - old_angle_MT6701 < 0.0)
    {
        relative_gear_rev = (360 - old_angle_MT6701 + angle_MT6701) / 360.0 / 0.05 ;
        old_angle_MT6701 = angle_MT6701;
    }
    else
    {
        relative_gear_rev = 0.0;
    }
    
    // if(SEND_TEST_MESSAGE)
    // {
    //     hal.scheduler->delay(10);
    //     gcs().send_text(MAV_SEVERITY_CRITICAL, "[5-2] angle_f: %.4f.", angle_f);
    //     hal.scheduler->delay(10);
    // }
    
    hal.scheduler->delay(10);
    gcs().send_text(MAV_SEVERITY_CRITICAL, "[5-2] angle_f: %.4f.", angle_f);
    gcs().send_text(MAV_SEVERITY_CRITICAL, "[5-3] gear_rev: %.4f.", relative_gear_rev);
    hal.scheduler->delay(10);
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
    else{
        if(SEND_TEST_MESSAGE)
        {
            hal.scheduler->delay(10);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "[6-3] read register 0x03 failed.");
            gcs().send_text(MAV_SEVERITY_CRITICAL, "[6-3-1] read_reg3: %02x.", read_reg3);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "[6-3-2] ReadBuffer: %d.", ReadBuffer);
            hal.scheduler->delay(10);
        }
    }
    if (_dev->transfer(&read_reg4, 1, &ReadBuffer, sizeof(ReadBuffer)))
    {
        angle += ReadBuffer;
        angle >>= 2;
        angle_f = (float)(angle * 360.0) / 16384.0;
        reading_m = angle_f;     
    }
    else{
        if(SEND_TEST_MESSAGE)
        {
            hal.scheduler->delay(10);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "[6-5] read register 0x04 failed.");
            gcs().send_text(MAV_SEVERITY_CRITICAL, "[6-5-1] read_reg4: %02x.", read_reg4);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "[6-5-2] ReadBuffer: %d.", ReadBuffer);
            hal.scheduler->delay(10);
        }
    }
}

/*
   update the state of the sensor
*/
void AP_Encoder_MT6701_I2C::update(void)
{
    // nothing to do - its all done in the timer()
}



