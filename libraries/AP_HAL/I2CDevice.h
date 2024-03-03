/*
 * Copyright (C) 2015-2016  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <inttypes.h>
#include <vector>

#include "AP_HAL_Namespace.h"
#include "Device.h"
#include "utility/OwnPtr.h"

namespace AP_HAL {

class I2CDevice : public Device {
public:
    I2CDevice() : Device(BUS_TYPE_I2C) { }

    virtual ~I2CDevice() { }

    /* Device implementation */

    /* See Device::set_speed() */
    virtual bool set_speed(Device::Speed speed) override = 0;

    /* See Device::transfer() */
    virtual bool transfer(const uint8_t *send, uint32_t send_len,
                          uint8_t *recv, uint32_t recv_len) override = 0;

    /*
     * Read location from device multiple times, advancing the buffer each
     * time
     * 多次从设备读取位置，每次都推进缓冲区
     */
    virtual bool read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                         uint32_t recv_len, uint8_t times) = 0;

    /* See Device::get_semaphore() */
    virtual Semaphore *get_semaphore() override = 0;

    /* See Device::register_periodic_callback() */
    virtual Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, Device::PeriodicCb) override = 0;

    /* See Device::adjust_periodic_callback() */
    virtual bool adjust_periodic_callback(
        Device::PeriodicHandle h, uint32_t period_usec) override = 0;

    /*
     * Force I2C transfers to be split between send and receive parts, with a
     * stop condition between them. Setting this allows to conveniently
     * continue using the read_* and transfer() methods on those devices.
     *
     * Some platforms may have transfers always split, in which case
     * this method is not needed.
     */
    virtual void set_split_transfers(bool set) {};
};

class I2CDeviceManager {
public:
    // 这段代码定义了两个get_device函数，它们都是虚函数（virtual），意味着它们可以在派生类中被重写（override）。
    // 这两个函数的目的都是获取一个指向I2C设备的指针，但是它们接受不同的参数集，并具有不同的实现。
    // 总的来说，这段代码提供了两种获取I2C设备对象的方法。
    // 第一种方法通过指定I2C总线和设备地址来获取设备，而第二种方法通过提供一系列可能的设备路径来获取设备。
    // 这两种方法的具体实现将取决于派生类，这些派生类可能针对不同的硬件平台或操作系统提供特定的实现。
    /* Get a device handle */
    // 参数：它接受四个参数——I2C总线编号（bus）、设备地址（address）、总线时钟频率（bus_clock，默认为400000 Hz）、是
    // 否使用SMBus协议（use_smbus，默认为false）和超时时间（timeout_ms，默认为4毫秒）。
    // 返回类型：返回一个OwnPtr<AP_HAL::I2CDevice>类型的对象。
    // OwnPtr可能是一个智能指针，用于自动管理I2C设备对象的生命周期（例如，当对象不再需要时自动释放内存）。
    // 实现：这个函数的实现被声明为= 0，意味着它是一个纯虚函数。
    // 这意味着在这个函数的声明所在的类中，它必须被声明为virtual，并且在这个类的任何派生类中都必须被重写（实现）。
    virtual OwnPtr<AP_HAL::I2CDevice> get_device(uint8_t bus, uint8_t address,
                                                 uint32_t bus_clock=400000,
                                                 bool use_smbus = false,
                                                 uint32_t timeout_ms=4) = 0;
    /*
     * Get device by looking up the I2C bus on the buses from @devpaths.
     *
     * Each string in @devpaths are possible locations for the bus. How the
     * strings are implemented are HAL-specific. On Linux this is the info
     * returned by 'udevadm info -q path /dev/i2c-X'. The first I2C bus
     * matching a prefix in @devpaths is used to create a I2CDevice object.
     */
    // 参数：它接受一个字符串向量（devpaths）和设备地址（address）。
    // devpaths可能包含一系列可能的I2C总线路径，这些路径将被用来查找和创建I2C设备对象。
    // 返回类型：返回一个OwnPtr<I2CDevice>类型的对象。
    // 注意这里的I2CDevice可能与第一个函数中的AP_HAL::I2CDevice不同，这取决于具体的实现和上下文。
    // 实现：这个函数的实现目前为空，只是返回了一个nullptr。
    // 这意味着这个函数目前没有被实现，调用它将返回一个空指针。
    // 这可能是因为这个函数在基类中被声明为虚函数，预期在派生类中被重写。
    virtual OwnPtr<I2CDevice> get_device(std::vector<const char *> devpaths,
                                         uint8_t address) {
        // Not implemented
        return nullptr;
    }

    /*
      get mask of bus numbers for all configured I2C buses
     */
    virtual uint32_t get_bus_mask(void) const { return 0x0F; }

    /*
      get mask of bus numbers for all configured external I2C buses
     */
    virtual uint32_t get_bus_mask_external(void) const { return 0x0F; }

    /*
      get mask of bus numbers for all configured internal I2C buses
     */
    virtual uint32_t get_bus_mask_internal(void) const { return 0x01; }
};

/*
  convenient macros for iterating over I2C bus numbers
 */
#define FOREACH_I2C_MASK(i,mask) for (uint32_t _bmask=mask, i=0; i<32; i++) if ((1U<<i)&_bmask)
#define FOREACH_I2C_EXTERNAL(i) FOREACH_I2C_MASK(i,hal.i2c_mgr->get_bus_mask_external())
#define FOREACH_I2C_INTERNAL(i) FOREACH_I2C_MASK(i,hal.i2c_mgr->get_bus_mask_internal())
#define FOREACH_I2C(i) FOREACH_I2C_MASK(i,hal.i2c_mgr->get_bus_mask())

}
