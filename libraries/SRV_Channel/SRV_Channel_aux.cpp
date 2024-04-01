/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  SRV_Channel_aux.cpp - handling of servo auxillary functions
 */
#include "SRV_Channel.h"

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <RC_Channel/RC_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_RangeFinder/AP_Encoder_MT6701_I2C.h"
// 这里引用了和 libraries 同级目录下其他文件夹中的头文件，所以用 "../"来指代
#include "../ArduPlane/Plane.h"

#if NUM_SERVO_CHANNELS == 0
#pragma GCC diagnostic ignored "-Wtype-limits"
#endif

extern const AP_HAL::HAL &hal;

#define MOTOR_STOP_VALUE 1000
#define MOTOR_STOP_DELAY_VALUE 1250
#define SERVO_BRAKE_VALUE 1850
#define SERVO_RELEASE_VALUE 1100
#define MOTOR_STOP false
#define MOTOR_RUN true
#define SERVO_BRAKE true
#define SERVO_RELEASE false
#define BREAK_DELAY_TIME_OFFSET_THRESHOLD 60
// static uint64_t current_time_4_us;
// static uint64_t stored_time_4_us;
uint16_t ch3_pwm_pid = 1250;
signed delta_ch3_pwm = 0;
// 编码器读取磁场角度的频率是 50Hz
// 控制油门输出的任务的执行频率是 400 Hz
// 因此采用一个计数变量使他们的频率能够对应上
uint8_t ch3_pwm_pid_counter = 0;
// static uint16_t ch4_pwm = 1300;
// static uint16_t ch8_pwm = 1000;
static bool Motor = true;  // true:电机正在运行，false:电机停止运行
static bool Servo = false; // true:舵机正在刹车，false:舵机停止刹车
static bool old_Glide_Mode_Flag = false;
float target_angle_MT6701 = 100;
float breaking_angle = 149.04;
float mag_angle_delay_time_ms = 200;
uint64_t current_break_time = 0;
static bool current_break_time_flag = false;
// 一下变量都记入日志 log_Encoder2 中
uint64_t break_time = 0;
uint64_t delay_time = 0;
uint64_t target_time = 0;
uint64_t current_time = 0;
uint64_t break_delta_time = 0;
bool delay_time_flag = false;
bool gear_rev_ready_flag = false;
bool mag_angle_delay_flag = false;
// 自动调节刹车等待时间的相关变量
uint8_t break_success_flag = 0;
float break_success_angle = 0.0;
float break_delay_time_offset = 0.0;
uint16_t break_delay_time_offset_counter = 0;
// 增量式PID控制器相关变量
float error_buff[3] = {0, 0, 0};
float K_p = 0.544491;
float K_i = 4.315097;
float K_d = 0.007933;

/// map a function to a servo channel and output it
void SRV_Channel::output_ch(void)
{
#ifndef HAL_BUILD_AP_PERIPH
    int8_t passthrough_from = -1;

    // take care of special function cases
    switch (function.get())
    {
    case k_manual: // manual
        passthrough_from = ch_num;
        break;
    case k_rcin1 ... k_rcin16: // rc pass-thru
        passthrough_from = int8_t((int16_t)function - k_rcin1);
        break;
    }
    if (passthrough_from != -1)
    {
        // we are doing passthrough from input to output for this channel
        RC_Channel *c = rc().channel(passthrough_from);
        if (c)
        {
            if (SRV_Channels::passthrough_disabled())
            {
                output_pwm = c->get_radio_trim();
            }
            else
            {
                const int16_t radio_in = c->get_radio_in();
                if (!ign_small_rcin_changes)
                {
                    output_pwm = radio_in;
                    previous_radio_in = radio_in;
                }
                else
                {
                    // check if rc input value has changed by more than the deadzone
                    if (abs(radio_in - previous_radio_in) > c->get_dead_zone())
                    {
                        output_pwm = radio_in;
                        ign_small_rcin_changes = false;
                    }
                }
            }
        }
    }
#endif // HAL_BUILD_AP_PERIPH

    if (!(SRV_Channels::disabled_mask & (1U << ch_num)))
    {
        if (Glide_Mode_Flag == true)
        {
            // 尝试通过 ch3_pwm 的值、测量到的齿轮转速、之前存储的齿轮转过的角度对等待延时时间和减速延时时间进行修正。
            // 先用1100到1400的 ch3_pwm 值进行测试，更高的值可以先暂时不用测。
            // breaking_angle 需要通过实验测得（可以通过日志检验），表示不同齿轮转速下的刹车所需角度
            // 不同转速下的 breaking_angle 值应该是不同的
            
            if (old_Glide_Mode_Flag == false)
            {
                old_Glide_Mode_Flag = true;
                ch3_pwm_pid = 1250;                                       
                /*
                 * 特别注意：控制电机输出的任务中不能使用延时函数！！！
                 * 因为控制电机输出的任务执行频率较高，如果在其中调用了延时函数，
                 * 程序会认为该任务卡死了，就会重启飞控程序，
                 * 导致飞控板与MP之间的连接中断，飞控程序不能正常运行。
                 * 这里的延时要采用其他的延时逻辑。
                 * */
                // hal.scheduler->delay(mag_angle_delay_time_ms);
            }

            if(break_success_flag == 2)
            {                
                break_success_flag = 3;
                if(break_success_angle - target_angle_MT6701 > 5)
                {
                    if(break_delay_time_offset_counter < 10)
                    {
                        break_delay_time_offset_counter++;
                        break_delay_time_offset = break_delay_time_offset - 5;
                        if(break_delay_time_offset + BREAK_DELAY_TIME_OFFSET_THRESHOLD < 0)
                        {
                            break_delay_time_offset = -BREAK_DELAY_TIME_OFFSET_THRESHOLD;
                        }
                    }
                    else
                    {
                        break_delay_time_offset_counter++;
                        break_delay_time_offset = break_delay_time_offset - 3;
                        if(break_delay_time_offset + BREAK_DELAY_TIME_OFFSET_THRESHOLD < 0)
                        {
                            break_delay_time_offset = -BREAK_DELAY_TIME_OFFSET_THRESHOLD;
                        }
                    }
                }
                else if(break_success_angle - target_angle_MT6701 < -5)
                {
                    if(break_delay_time_offset_counter < 10)
                    {
                        break_delay_time_offset_counter++;
                        break_delay_time_offset = break_delay_time_offset + 5;
                        if(break_delay_time_offset - BREAK_DELAY_TIME_OFFSET_THRESHOLD > 0)
                        {
                            break_delay_time_offset = BREAK_DELAY_TIME_OFFSET_THRESHOLD;
                        }
                    }
                    else
                    {
                        break_delay_time_offset_counter++;
                        break_delay_time_offset = break_delay_time_offset + 3;
                        if(break_delay_time_offset - BREAK_DELAY_TIME_OFFSET_THRESHOLD > 0)
                        {
                            break_delay_time_offset = BREAK_DELAY_TIME_OFFSET_THRESHOLD;
                        }
                    }
                }
            }
            /*
             * 注意：此处必须是以下的判断形式
             * if(舵机通道){}
             * else if(电机通道){}
             * else{其他通道}
             * 或
             * if(电机通道){}
             * else if(舵机通道){}
             * else{其他通道}
             *
             * 错误写法【1】
             * if(舵机通道){}
             * if(电机通道){}
             * else{其他通道}
             * 错误写法【2】
             * if(电机通道){}
             * if(舵机通道){}
             * else{其他通道}
             * 如果分成一个if语句和一个if else语句来判断输入通道的话
             * 对于错误写法【1】，当舵机通道执行output_ch()时，会进入if(舵机通道)语句
             * 但同时也会进入else{其他通道}语句，此时舵机通道就不再受持续控制
             * 对于错误写法【2】，当电机通道执行output_ch()时，会进入if(电机通道)语句
             * 但同时也会进入else{其他通道}语句，此时电机通道就不再受持续控制
             * */
            if (ch_num == 0) // 制动舵机
            {
                // 驱动舵机臂刹车
                if ((Motor == MOTOR_STOP) && (Servo == SERVO_RELEASE))
                {
                    Servo = SERVO_BRAKE;
                    hal.rcout->write(ch_num, SERVO_BRAKE_VALUE);

                    break_success_flag = 1;
                }
            }
            else if (ch_num == 2) // 驱动电机
            {
                // 电机停转
                if ((Motor == MOTOR_RUN) && (Servo == SERVO_RELEASE))
                {            
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "avg_relative_gear_rev: %.2f.", avg_relative_gear_rev);        
                    if ((abs(avg_relative_gear_rev - 5.0) < 0.2) && (mag_angle_delay_flag == false))
                    {
                        gear_rev_ready_flag = true;
                        break_angle_MT6701 = angle_MT6701;
                    }
                    else if(((avg_relative_gear_rev - 5.2) > 0) && (mag_angle_delay_flag == false))
                    {
                        // 对这里的转速控制采用PID控制器
                        for(uint8_t i = 0; i < 2; i++)
                        {
                            error_buff[i] = error_buff[i+1];
                        }
                        error_buff[2] = 5.0 - avg_relative_gear_rev;
                        delta_ch3_pwm = (signed)(K_p * (error_buff[2] - error_buff[1]) + K_i * error_buff[2] + K_d * (error_buff[2] - 2 * error_buff[1] + error_buff[0]));
                        ch3_pwm_pid_counter++;
                        if(ch3_pwm_pid_counter >= 8)
                        {
                            ch3_pwm_pid_counter = 0;
                            ch3_pwm_pid = ch3_pwm_pid + delta_ch3_pwm;
                            if(ch3_pwm_pid > 1350)
                            {
                                ch3_pwm_pid = 1350;
                            }
                            else if(ch3_pwm_pid < 1200)
                            {
                                ch3_pwm_pid = 1200;
                            }
                        }

                        hal.rcout->write(ch_num, ch3_pwm_pid);
                    }
                    else if(((avg_relative_gear_rev - 4.8) < 0) && (mag_angle_delay_flag == false))
                    {
                        // 对这里的转速控制采用PID控制器
                        for(uint8_t i = 0; i < 2; i++)
                        {
                            error_buff[i] = error_buff[i+1];
                        }
                        error_buff[2] = 5.0 - avg_relative_gear_rev;
                        delta_ch3_pwm = (signed)(K_p * (error_buff[2] - error_buff[1]) + K_i * error_buff[2] + K_d * (error_buff[2] - 2 * error_buff[1] + error_buff[0]));
                        ch3_pwm_pid_counter++;
                        if(ch3_pwm_pid_counter >= 8)
                        {
                            ch3_pwm_pid_counter = 0;
                            ch3_pwm_pid = ch3_pwm_pid + delta_ch3_pwm;
                            if(ch3_pwm_pid > 1350)
                            {
                                ch3_pwm_pid = 1350;
                            }
                            else if(ch3_pwm_pid < 1200)
                            {
                                ch3_pwm_pid = 1200;
                            }
                        }

                        hal.rcout->write(ch_num, ch3_pwm_pid);
                    }
                    else
                    {
                        hal.rcout->write(ch_num, MOTOR_STOP_DELAY_VALUE);
                    }

                    if(gear_rev_ready_flag == true)
                    {
                        gear_rev_ready_flag = false;
                        current_break_time_flag = true;
                        mag_angle_delay_flag = true;

                        // 情况一
                        if ((target_angle_MT6701 - break_angle_MT6701) >= 0)
                        {
                            // 情况二
                            // 目标角度减去当前齿轮角度，再减去刹车所需预留角度都还要大于0
                            // 说明需要让齿轮保持当前速度并等待一定时间
                            if ((target_angle_MT6701 - (break_angle_MT6701 + breaking_angle)) > 0)
                            {                  
                                // 注意，这里计算出来的单位是秒，乘以1000后得到的数字单位才是毫秒。          
                                mag_angle_delay_time_ms = (target_angle_MT6701 - break_angle_MT6701 - breaking_angle) / 360 / avg_relative_gear_rev * 1000 + break_delay_time_offset;
                            }
                            // 情况三
                            // 目标角度减去当前齿轮角度，再减去刹车所需预留角度小于0时
                            // 说明需要让齿轮多转一圈，才能预留出足够的刹车所需角度
                            else
                            {                            
                                mag_angle_delay_time_ms = (target_angle_MT6701 + 360 - break_angle_MT6701 - breaking_angle) / 360 / avg_relative_gear_rev * 1000 + break_delay_time_offset;
                            }
                        }
                        // 情况四
                        else
                        {
                            // 情况五
                            // 目标角度减去当前齿轮角度，再减去刹车所需预留角度都还要大于0
                            // 说明需要让齿轮保持当前速度并等待一定时间
                            if ((360 - break_angle_MT6701 + target_angle_MT6701 - breaking_angle) > 0)
                            {                            
                                mag_angle_delay_time_ms = (target_angle_MT6701 + 360 - break_angle_MT6701 - breaking_angle) / 360 / avg_relative_gear_rev * 1000 + break_delay_time_offset;
                            }
                            // 情况六
                            // 目标角度减去当前齿轮角度，再减去刹车所需预留角度小于0时
                            // 说明需要让齿轮多转一圈，才能预留出足够的刹车所需角度
                            else
                            {                            
                                mag_angle_delay_time_ms = (target_angle_MT6701 + 720 - break_angle_MT6701 - breaking_angle) / 360 / avg_relative_gear_rev * 1000 + break_delay_time_offset;
                            }
                        }
                    }

                    if(current_break_time_flag == true)
                    {
                        current_break_time_flag = false;
                        current_break_time = AP_HAL::micros64();                        
                    }
                    if((mag_angle_delay_time_ms - 1000 < 0.0) && (mag_angle_delay_flag == true))
                    {
                        if(AP_HAL::micros64() >= (current_break_time + (uint64_t)(mag_angle_delay_time_ms * 1000)))
                        {
                            delay_time_flag = true;
                            mag_angle_delay_flag = false;
                            break_time = current_break_time;
                            delay_time = (uint64_t)(mag_angle_delay_time_ms * 1000);
                            target_time = current_break_time + (uint64_t)(mag_angle_delay_time_ms * 1000);
                            current_time = AP_HAL::micros64();
                            break_delta_time = AP_HAL::micros64() - current_break_time;

                            Motor = MOTOR_STOP;
                            hal.rcout->write(ch_num, MOTOR_STOP_VALUE);
                        }
                        else
                        {
                            delay_time_flag = false;
                            break_time = current_break_time;
                            delay_time = (uint64_t)(mag_angle_delay_time_ms * 1000);
                            target_time = current_break_time + (uint64_t)(mag_angle_delay_time_ms * 1000);
                            current_time = AP_HAL::micros64();
                            break_delta_time = AP_HAL::micros64() - current_break_time;

                            hal.rcout->write(ch_num, ch3_pwm);
                        }   
                    }
                }
            }
            else // 其他PWM通道
            {
                hal.rcout->write(ch_num, output_pwm);
            }
        }
        else
        {
            old_Glide_Mode_Flag = Glide_Mode_Flag;
            if (ch_num == 0) // 制动舵机
            {
                if ((Motor == MOTOR_STOP) && (Servo == SERVO_BRAKE))
                {
                    Servo = SERVO_RELEASE;
                    hal.rcout->write(ch_num, SERVO_RELEASE_VALUE);
                }
            }
            else if (ch_num == 2) // 驱动电机
            {
                if ((Motor == MOTOR_STOP) && (Servo == SERVO_RELEASE))
                {
                    Motor = MOTOR_RUN;
                    hal.rcout->write(ch_num, output_pwm);
                }
                else if ((Motor == MOTOR_RUN) && (Servo == SERVO_RELEASE))
                {                 
                    hal.rcout->write(ch_num, output_pwm);
                }
            }
            else
            {
                hal.rcout->write(ch_num, output_pwm);
            }
        }
    }
}

/*
  call output_ch() on all channels
 */
void SRV_Channels::output_ch_all(void)
{
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++)
    {
        channels[i].output_ch();
    }
}

/*
  return the current function for a channel
*/
SRV_Channel::Aux_servo_function_t SRV_Channels::channel_function(uint8_t channel)
{
    if (channel < NUM_SERVO_CHANNELS)
    {
        return channels[channel].function;
    }
    return SRV_Channel::k_none;
}

/*
   setup a channels aux servo function
*/
void SRV_Channel::aux_servo_function_setup(void)
{
    if (type_setup)
    {
        return;
    }
    switch (function.get())
    {
    case k_flap:
    case k_flap_auto:
    case k_egg_drop:
        set_range(100);
        break;
    case k_heli_rsc:
    case k_heli_tail_rsc:
    case k_motor_tilt:
    case k_boost_throttle:
    case k_thrust_out:
        set_range(1000);
        break;
    case k_aileron_with_input:
    case k_elevator_with_input:
    case k_aileron:
    case k_elevator:
    case k_dspoilerLeft1:
    case k_dspoilerLeft2:
    case k_dspoilerRight1:
    case k_dspoilerRight2:
    case k_rudder:
    case k_steering:
    case k_flaperon_left:
    case k_flaperon_right:
    case k_tiltMotorLeft:
    case k_tiltMotorRight:
    case k_tiltMotorRear:
    case k_tiltMotorRearLeft:
    case k_tiltMotorRearRight:
    case k_elevon_left:
    case k_elevon_right:
    case k_vtail_left:
    case k_vtail_right:
    case k_scripting1:
    case k_scripting2:
    case k_scripting3:
    case k_scripting4:
    case k_scripting5:
    case k_scripting6:
    case k_scripting7:
    case k_scripting8:
    case k_scripting9:
    case k_scripting10:
    case k_scripting11:
    case k_scripting12:
    case k_scripting13:
    case k_scripting14:
    case k_scripting15:
    case k_scripting16:
    case k_roll_out:
    case k_pitch_out:
    case k_yaw_out:
        set_angle(4500);
        break;
    case k_throttle:
    case k_throttleLeft:
    case k_throttleRight:
    case k_airbrake:
        // fixed wing throttle
        set_range(100);
        break;
    default:
        break;
    }
}

/// setup the output range types of all functions
void SRV_Channels::update_aux_servo_function(void)
{
    if (!channels)
    {
        return;
    }
    function_mask.clearall();

    for (uint16_t i = 0; i < SRV_Channel::k_nr_aux_servo_functions; i++)
    {
        functions[i].channel_mask = 0;
    }

    // set auxiliary ranges
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++)
    {
        if (!channels[i].valid_function())
        {
            continue;
        }
        const uint16_t function = channels[i].function.get();
        channels[i].aux_servo_function_setup();
        function_mask.set(function);
        functions[function].channel_mask |= 1U << i;
    }
    initialised = true;
}

/// Should be called after the the servo functions have been initialized
/// called at 1Hz
void SRV_Channels::enable_aux_servos()
{
    hal.rcout->set_default_rate(uint16_t(_singleton->default_rate.get()));

    update_aux_servo_function();

    // enable all channels that are set to a valid function. This
    // includes k_none servos, which allows those to get their initial
    // trim value on startup
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++)
    {
        SRV_Channel &c = channels[i];
        // see if it is a valid function
        if (c.valid_function() && !(disabled_mask & (1U << c.ch_num)))
        {
            hal.rcout->enable_ch(c.ch_num);
        }
        else
        {
            hal.rcout->disable_ch(c.ch_num);
        }

        // output some servo functions before we fiddle with the
        // parameter values:
        if (c.function == SRV_Channel::k_min)
        {
            c.set_output_pwm(c.servo_min);
            c.output_ch();
        }
        else if (c.function == SRV_Channel::k_trim)
        {
            c.set_output_pwm(c.servo_trim);
            c.output_ch();
        }
        else if (c.function == SRV_Channel::k_max)
        {
            c.set_output_pwm(c.servo_max);
            c.output_ch();
        }
    }

    // propagate channel masks to the ESCS
    hal.rcout->update_channel_masks();

#if HAL_SUPPORT_RCOUT_SERIAL
    blheli_ptr->update();
#endif
}

/*
    for channels which have been marked as digital output then the
    MIN/MAX/TRIM values have no meaning for controlling output, as
    the HAL handles the scaling. We still need to cope with places
    in the code that may try to set a PWM value however, so to
    ensure consistency we force the MIN/MAX/TRIM to be consistent
    across all digital channels. We use a MIN/MAX of 1000/2000, and
    set TRIM to either 1000 or 1500 depending on whether the channel
    is reversible
*/
void SRV_Channels::set_digital_outputs(uint16_t dig_mask, uint16_t rev_mask)
{
    digital_mask |= dig_mask;
    reversible_mask |= rev_mask;

    // add in NeoPixel and ProfiLED functions to digital array to determine anything else
    // that should be disabled
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++)
    {
        SRV_Channel &c = channels[i];
        switch (c.function.get())
        {
        case SRV_Channel::k_LED_neopixel1:
        case SRV_Channel::k_LED_neopixel2:
        case SRV_Channel::k_LED_neopixel3:
        case SRV_Channel::k_LED_neopixel4:
        case SRV_Channel::k_ProfiLED_1:
        case SRV_Channel::k_ProfiLED_2:
        case SRV_Channel::k_ProfiLED_3:
        case SRV_Channel::k_ProfiLED_Clock:
            dig_mask |= 1U << c.ch_num;
            break;
        default:
            break;
        }
    }
    disabled_mask = hal.rcout->get_disabled_channels(dig_mask);

    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++)
    {
        SRV_Channel &c = channels[i];
        if (digital_mask & (1U << i))
        {
            c.servo_min.set(1000);
            c.servo_max.set(2000);
            if (reversible_mask & (1U << i))
            {
                c.servo_trim.set(1500);
            }
            else
            {
                c.servo_trim.set(1000);
            }
        }
    }
}

/// enable output channels using a channel mask
void SRV_Channels::enable_by_mask(uint16_t mask)
{
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++)
    {
        if (mask & (1U << i))
        {
            hal.rcout->enable_ch(i);
        }
    }
}

/*
  set radio_out for all channels matching the given function type
 */
void SRV_Channels::set_output_pwm(SRV_Channel::Aux_servo_function_t function, uint16_t value)
{
    if (!function_assigned(function))
    {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++)
    {
        if (channels[i].function == function)
        {
            channels[i].set_output_pwm(value);
            channels[i].output_ch();
        }
    }
}

/*
  set radio_out for all channels matching the given function type
  trim the output assuming a 1500 center on the given value
  reverses pwm output based on channel reversed property
 */
void SRV_Channels::set_output_pwm_trimmed(SRV_Channel::Aux_servo_function_t function, int16_t value)
{
    if (!function_assigned(function))
    {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++)
    {
        if (channels[i].function == function)
        {
            int16_t value2;
            if (channels[i].get_reversed())
            {
                value2 = 1500 - value + channels[i].get_trim();
            }
            else
            {
                value2 = value - 1500 + channels[i].get_trim();
            }
            channels[i].set_output_pwm(constrain_int16(value2, channels[i].get_output_min(), channels[i].get_output_max()));
            channels[i].output_ch();
        }
    }
}

/*
  set and save the trim value to current output for all channels matching
  the given function type
 */
void SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::Aux_servo_function_t function)
{
    if (!function_assigned(function))
    {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++)
    {
        if (channels[i].function == function)
        {
            channels[i].servo_trim.set_and_save_ifchanged(channels[i].get_output_pwm());
        }
    }
}

/*
  copy radio_in to radio_out for a given function
 */
void SRV_Channels::copy_radio_in_out(SRV_Channel::Aux_servo_function_t function, bool do_input_output)
{
    if (!function_assigned(function))
    {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++)
    {
        if (channels[i].function == function)
        {
            RC_Channel *c = rc().channel(channels[i].ch_num);
            if (c == nullptr)
            {
                continue;
            }
            channels[i].set_output_pwm(c->get_radio_in());
            if (do_input_output)
            {
                channels[i].output_ch();
            }
        }
    }
}

/*
  copy radio_in to radio_out for a channel mask
 */
void SRV_Channels::copy_radio_in_out_mask(uint16_t mask)
{
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++)
    {
        if ((1U << i) & mask)
        {
            RC_Channel *c = rc().channel(channels[i].ch_num);
            if (c == nullptr)
            {
                continue;
            }
            channels[i].set_output_pwm(c->get_radio_in());
        }
    }
}

/*
  setup failsafe value for an auxiliary function type to a Limit
 */
void SRV_Channels::set_failsafe_pwm(SRV_Channel::Aux_servo_function_t function, uint16_t pwm)
{
    if (!function_assigned(function))
    {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++)
    {
        const SRV_Channel &c = channels[i];
        if (c.function == function)
        {
            hal.rcout->set_failsafe_pwm(1U << c.ch_num, pwm);
        }
    }
}

/*
  setup failsafe value for an auxiliary function type to a Limit
 */
void SRV_Channels::set_failsafe_limit(SRV_Channel::Aux_servo_function_t function, SRV_Channel::Limit limit)
{
    if (!function_assigned(function))
    {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++)
    {
        const SRV_Channel &c = channels[i];
        if (c.function == function)
        {
            uint16_t pwm = c.get_limit_pwm(limit);
            hal.rcout->set_failsafe_pwm(1U << c.ch_num, pwm);
        }
    }
}

/*
  set radio output value for an auxiliary function type to a Limit
 */
void SRV_Channels::set_output_limit(SRV_Channel::Aux_servo_function_t function, SRV_Channel::Limit limit)
{
    if (!function_assigned(function))
    {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++)
    {
        SRV_Channel &c = channels[i];
        if (c.function == function)
        {
            uint16_t pwm = c.get_limit_pwm(limit);
            c.set_output_pwm(pwm);
            if (c.function == SRV_Channel::k_manual)
            {
                RC_Channel *cin = rc().channel(c.ch_num);
                if (cin != nullptr)
                {
                    // in order for output_ch() to work for k_manual we
                    // also have to override radio_in
                    cin->set_radio_in(pwm);
                }
            }
        }
    }
}

/*
  return true if a particular function is assigned to at least one RC channel
 */
bool SRV_Channels::function_assigned(SRV_Channel::Aux_servo_function_t function)
{
    if (!initialised)
    {
        update_aux_servo_function();
    }
    return function_mask.get(uint16_t(function));
}

/*
  set servo_out and angle_min/max, then calc_pwm and output a
  value. This is used to move a AP_Mount servo
 */
void SRV_Channels::move_servo(SRV_Channel::Aux_servo_function_t function,
                              int16_t value, int16_t angle_min, int16_t angle_max)
{
    if (!function_assigned(function))
    {
        return;
    }
    if (angle_max <= angle_min)
    {
        return;
    }
    float v = float(value - angle_min) / float(angle_max - angle_min);
    v = constrain_float(v, 0.0f, 1.0f);
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++)
    {
        SRV_Channel &c = channels[i];
        if (c.function == function)
        {
            float v2 = c.get_reversed() ? (1 - v) : v;
            uint16_t pwm = c.servo_min + v2 * (c.servo_max - c.servo_min);
            c.set_output_pwm(pwm);
        }
    }
}

/*
  set the default channel an auxiliary output function should be on
 */
bool SRV_Channels::set_aux_channel_default(SRV_Channel::Aux_servo_function_t function, uint8_t channel)
{
    if (function_assigned(function))
    {
        // already assigned
        return true;
    }
    if (channels[channel].function != SRV_Channel::k_none)
    {
        if (channels[channel].function == function)
        {
            return true;
        }
        hal.console->printf("Channel %u already assigned function %u\n",
                            (unsigned)(channel + 1),
                            (unsigned)channels[channel].function.get());
        return false;
    }
    channels[channel].type_setup = false;
    channels[channel].function.set(function);
    channels[channel].aux_servo_function_setup();
    function_mask.set((uint16_t)function);
    if (SRV_Channel::valid_function(function))
    {
        functions[function].channel_mask |= 1U << channel;
    }
    return true;
}

// find first channel that a function is assigned to
bool SRV_Channels::find_channel(SRV_Channel::Aux_servo_function_t function, uint8_t &chan)
{
    if (!function_assigned(function))
    {
        return false;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++)
    {
        if (channels[i].function == function)
        {
            chan = channels[i].ch_num;
            return true;
        }
    }
    return false;
}

/*
  get a pointer to first auxillary channel for a channel function
*/
SRV_Channel *SRV_Channels::get_channel_for(SRV_Channel::Aux_servo_function_t function, int8_t default_chan)
{
    uint8_t chan;
    if (default_chan >= 0)
    {
        set_aux_channel_default(function, default_chan);
    }
    if (!find_channel(function, chan))
    {
        return nullptr;
    }
    return &channels[chan];
}

void SRV_Channels::set_output_scaled(SRV_Channel::Aux_servo_function_t function, float value)
{
    if (SRV_Channel::valid_function(function))
    {
        functions[function].output_scaled = value;
        SRV_Channel::have_pwm_mask &= ~functions[function].channel_mask;
    }
}

float SRV_Channels::get_output_scaled(SRV_Channel::Aux_servo_function_t function)
{
    if (SRV_Channel::valid_function(function))
    {
        return functions[function].output_scaled;
    }
    return 0;
}

// get slew limited scaled output for the given function type
float SRV_Channels::get_slew_limited_output_scaled(SRV_Channel::Aux_servo_function_t function)
{
    if (!SRV_Channel::valid_function(function))
    {
        return 0.0;
    }
    for (slew_list *slew = _slew; slew; slew = slew->next)
    {
        if (slew->func == function)
        {
            if (!is_positive(slew->max_change))
            {
                // treat negative or zero slew rate as disabled
                break;
            }
            return constrain_float(functions[function].output_scaled, slew->last_scaled_output - slew->max_change, slew->last_scaled_output + slew->max_change);
        }
    }
    // no slew limiting
    return functions[function].output_scaled;
}

/*
  get mask of output channels for a function
 */
uint16_t SRV_Channels::get_output_channel_mask(SRV_Channel::Aux_servo_function_t function)
{
    if (!initialised)
    {
        update_aux_servo_function();
    }
    if (SRV_Channel::valid_function(function))
    {
        return functions[function].channel_mask;
    }
    return 0;
}

// set the trim for a function channel to given pwm
void SRV_Channels::set_trim_to_pwm_for(SRV_Channel::Aux_servo_function_t function, int16_t pwm)
{
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++)
    {
        if (channels[i].function == function)
        {
            channels[i].servo_trim.set(pwm);
        }
    }
}

// set the trim for a function channel to min output of the channel honnoring reverse unless ignore_reversed is true
void SRV_Channels::set_trim_to_min_for(SRV_Channel::Aux_servo_function_t function, bool ignore_reversed)
{
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++)
    {
        if (channels[i].function == function)
        {
            channels[i].servo_trim.set((channels[i].get_reversed() && !ignore_reversed) ? channels[i].servo_max : channels[i].servo_min);
        }
    }
}

/*
  set the default function for a channel
*/
void SRV_Channels::set_default_function(uint8_t chan, SRV_Channel::Aux_servo_function_t function)
{
    if (chan < NUM_SERVO_CHANNELS)
    {
        const SRV_Channel::Aux_servo_function_t old = channels[chan].function;
        channels[chan].function.set_default(function);
        if (old != channels[chan].function && channels[chan].function == function)
        {
            function_mask.set((uint16_t)function);
        }
    }
}

void SRV_Channels::set_esc_scaling_for(SRV_Channel::Aux_servo_function_t function)
{
    uint8_t chan;
    if (find_channel(function, chan))
    {
        hal.rcout->set_esc_scaling(channels[chan].get_output_min(), channels[chan].get_output_max());
    }
}

/*
  auto-adjust channel trim from an integrator value. Positive v means
  adjust trim up. Negative means decrease
 */
void SRV_Channels::adjust_trim(SRV_Channel::Aux_servo_function_t function, float v)
{
    if (is_zero(v))
    {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++)
    {
        SRV_Channel &c = channels[i];
        if (function != c.function)
        {
            continue;
        }
        float change = c.reversed ? -v : v;
        uint16_t new_trim = c.servo_trim;
        if (c.servo_max <= c.servo_min)
        {
            continue;
        }
        float trim_scaled = float(c.servo_trim - c.servo_min) / (c.servo_max - c.servo_min);
        if (change > 0 && trim_scaled < 0.6f)
        {
            new_trim++;
        }
        else if (change < 0 && trim_scaled > 0.4f)
        {
            new_trim--;
        }
        else
        {
            return;
        }
        c.servo_trim.set(new_trim);

        trimmed_mask |= 1U << i;
    }
}

// get pwm output for the first channel of the given function type.
bool SRV_Channels::get_output_pwm(SRV_Channel::Aux_servo_function_t function, uint16_t &value)
{
    uint8_t chan;
    if (!find_channel(function, chan))
    {
        return false;
    }
    if (!SRV_Channel::valid_function(function))
    {
        return false;
    }
    channels[chan].calc_pwm(functions[function].output_scaled);
    value = channels[chan].get_output_pwm();
    return true;
}

// set output pwm to trim for the given function
void SRV_Channels::set_output_to_trim(SRV_Channel::Aux_servo_function_t function)
{
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++)
    {
        if (channels[i].function == function)
        {
            channels[i].set_output_pwm(channels[i].servo_trim);
        }
    }
}

/*
  get the normalised output for a channel function from the pwm value
  of the first matching channel
 */
float SRV_Channels::get_output_norm(SRV_Channel::Aux_servo_function_t function)
{
    uint8_t chan;
    if (!find_channel(function, chan))
    {
        return 0;
    }
    if (SRV_Channel::valid_function(function))
    {
        channels[chan].calc_pwm(functions[function].output_scaled);
    }
    return channels[chan].get_output_norm();
}

// set normalised output (-1 to 1 with 0 at mid point of servo_min/servo_max) for the given function
void SRV_Channels::set_output_norm(SRV_Channel::Aux_servo_function_t function, float value)
{
    if (!function_assigned(function))
    {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++)
    {
        SRV_Channel &c = channels[i];
        if (c.function == function)
        {
            c.set_output_norm(value);
        }
    }
}

/*
  limit slew rate for an output function to given rate in percent per
  second. This assumes output has not yet done to the hal
 */
void SRV_Channels::set_slew_rate(SRV_Channel::Aux_servo_function_t function, float slew_rate, uint16_t range, float dt)
{
    if (!SRV_Channel::valid_function(function))
    {
        return;
    }
    const float max_change = range * slew_rate * 0.01 * dt;

    for (slew_list *slew = _slew; slew; slew = slew->next)
    {
        if (slew->func == function)
        {
            // found existing item, update max change
            slew->max_change = max_change;
            return;
        }
    }

    if (!is_positive(max_change))
    {
        // no point in adding a disabled slew limit
        return;
    }

    // add new item
    slew_list *new_slew = new slew_list(function);
    if (new_slew == nullptr)
    {
        return;
    }
    new_slew->last_scaled_output = functions[function].output_scaled;
    new_slew->max_change = max_change;
    new_slew->next = _slew;
    _slew = new_slew;
}

// call set_angle() on matching channels
void SRV_Channels::set_angle(SRV_Channel::Aux_servo_function_t function, uint16_t angle)
{
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++)
    {
        if (channels[i].function == function)
        {
            channels[i].set_angle(angle);
        }
    }
}

// call set_range() on matching channels
void SRV_Channels::set_range(SRV_Channel::Aux_servo_function_t function, uint16_t range)
{
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++)
    {
        if (channels[i].function == function)
        {
            channels[i].set_range(range);
        }
    }
}

// set MIN parameter for a function
void SRV_Channels::set_output_min_max(SRV_Channel::Aux_servo_function_t function, uint16_t min_pwm, uint16_t max_pwm)
{
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++)
    {
        if (channels[i].function == function)
        {
            channels[i].set_output_min(min_pwm);
            channels[i].set_output_max(max_pwm);
        }
    }
}

// constrain to output min/max for function
void SRV_Channels::constrain_pwm(SRV_Channel::Aux_servo_function_t function)
{
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++)
    {
        SRV_Channel &c = channels[i];
        if (c.function == function)
        {
            c.set_output_pwm(constrain_int16(c.output_pwm, c.servo_min, c.servo_max));
        }
    }
}

/*
  upgrade SERVO* parameters. This does the following:

   - update to 16 bit FUNCTION from AP_Int8
*/
void SRV_Channels::upgrade_parameters(void)
{
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++)
    {
        SRV_Channel &c = channels[i];
        // convert from AP_Int8 to AP_Int16
        c.function.convert_parameter_width(AP_PARAM_INT8);
    }
}

// set RC output frequency on a function output
void SRV_Channels::set_rc_frequency(SRV_Channel::Aux_servo_function_t function, uint16_t frequency_hz)
{
    uint16_t mask = 0;
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++)
    {
        SRV_Channel &c = channels[i];
        if (c.function == function)
        {
            mask |= (1U << c.ch_num);
        }
    }
    if (mask != 0)
    {
        hal.rcout->set_freq(mask, frequency_hz);
    }
}
