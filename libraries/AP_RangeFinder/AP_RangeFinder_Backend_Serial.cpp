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
 * 这里主要是初始化端口和波特率，还有更新读取的数据，
 * AP_RangeFinder_Backend_Serial::update() 函数就是在AP_RangeFinder.cpp中被调用的，
 * 在update() 函数中会调用 get_reading() 函数，这里的 get_reading() 是一个接口，
 * 就是第二节AP_RangeFinder_TeraRanger_Serial中实现的，到这里就完成了串口传感器的读取。
 * 
 * 这个文件里面就是实现了基类中的get_reading接口，在该接口实现具体针对TeraRanger的业务逻辑，
 * 也是通过基类uart指针调用UARTDriver类的成员来对串口进行读取等操作
 * 
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_Backend_Serial.h"
#include <AP_SerialManager/AP_SerialManager.h>

#include <ctype.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial(
    RangeFinder::RangeFinder_State &_state,
    AP_RangeFinder_Params &_params) :
    AP_RangeFinder_Backend(_state, _params)
{

}

// AP_RangeFinder_Backend_Serial::init_serial 函数是用于初始化串行通信接口的。
// 这个函数是 AP_RangeFinder_Backend_Serial 类的一部分，该类通常代表一个通过串行接口与主机通信的测距仪后端。
// 函数的参数 serial_instance 指定了要使用的串行接口的实例。
// 总体来说，AP_RangeFinder_Backend_Serial::init_serial 函数的目的是设置测距仪后端所使用的串行接口，
// 以便它可以与主机进行通信。这通常包括配置串行接口的波特率、数据位、停止位等参数，
// 以及设置接收和发送缓冲区的大小。
void AP_RangeFinder_Backend_Serial::init_serial(uint8_t serial_instance)
{
    // 1.查找串行接口：
    // 这行代码调用了全局的串行管理器 AP::serialmanager() 的 find_serial 方法来查找一个特定的串行接口。
    // 它指定了协议类型为 AP_SerialManager::SerialProtocol_Rangefinder，这通常表示该串行接口将用于与测距仪通信。
    // serial_instance 参数则指定了具体使用哪个串行接口的实例。
    uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance);
    
    // 2.初始化串行接口：
    // 如果找到了有效的串行接口（即 uart 不为 nullptr），则调用该接口的 begin 方法来初始化它。
    // begin 方法通常用于配置串行通信的参数，
    // 如波特率、接收缓冲区大小（rx_bufsize() 返回的值）和发送缓冲区大小（tx_bufsize() 返回的值）。
    // initial_baudrate(serial_instance) 是一个函数调用，它返回针对给定串行实例的初始波特率配置。
    if (uart != nullptr) {
        uart->begin(initial_baudrate(serial_instance), rx_bufsize(), tx_bufsize());
    }
}

uint32_t AP_RangeFinder_Backend_Serial::initial_baudrate(const uint8_t serial_instance) const
{
    return AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance);
}

/*
   detect if a Serial rangefinder is connected. We'll detect by simply
   checking for SerialManager configuration
*/
bool AP_RangeFinder_Backend_Serial::detect(uint8_t serial_instance)
{
    return AP::serialmanager().have_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance);
}


/*
   update the state of the sensor
*/
// 这个 update 函数是一个成员函数，属于 AP_RangeFinder_Backend_Serial 类，用于更新测距仪的状态。
// 它首先尝试获取测距仪的读数，然后根据读数的结果和超时时间来更新状态。
// 如果成功获取了读数，它会更新最后一次读取的时间并调用 update_status 函数；
// 如果超过了读取超时的时间限制而没有获取到有效的读数，它会将状态设置为 NoData。
void AP_RangeFinder_Backend_Serial::update(void)
{
    // 这一行调用 get_reading 函数来尝试从测距仪获取一个距离读数。
    // state.distance_m 是一个成员变量，用于存储测得的距离值（单位为米）。
    // get_reading 函数的具体实现会依赖于测距仪的硬件和通信协议。
    // 它通常会尝试从硬件接口（如串行端口）读取数据，并解析这些数据以获取距离值。
    // 如果 get_reading 函数成功获取了一个有效的距离读数，它会返回 true，然后执行 if 语句块内的代码。
    if (get_reading(state.distance_m)) {
        // update range_valid state based on distance measured
        // 如果成功获取了读数，这一行会更新 state.last_reading_ms 变量，将其设置为当前的时间（以毫秒为单位）。
        // AP_HAL::millis() 是一个函数，返回自系统启动以来的毫秒数，通常用于获取当前时间。
        state.last_reading_ms = AP_HAL::millis();
        // 在成功获取读数后，调用 update_status 函数来更新测距仪的状态。
        // 这个函数的具体实现可能会根据读数的有效性以及其他条件来确定状态（例如，是否连接、是否有数据等）。
        update_status();
    } 
    // 如果 get_reading 函数没有成功获取一个有效的距离读数（返回 false），
    // 则执行 else if 语句块内的代码。
    // 这一行计算自上一次成功获取读数以来的时间差（以毫秒为单位）。
    // 如果这个时间差大于 read_timeout_ms() 函数返回的值（即读取超时的时间限制），
    // 则执行 else if 语句块内的代码。
    else if (AP_HAL::millis() - state.last_reading_ms > read_timeout_ms()) {
        // 如果自上一次成功获取读数以来的时间超过了读取超时的时间限制，
        // 则调用 set_status 函数来设置测距仪的状态为 RangeFinder::Status::NoData。
        // 这表示测距仪目前没有数据可供读取。
        set_status(RangeFinder::Status::NoData);
    }
}
