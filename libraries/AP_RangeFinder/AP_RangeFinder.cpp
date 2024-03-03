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

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_analog.h"
#include "AP_RangeFinder_PulsedLightLRF.h"
#include "AP_RangeFinder_MaxsonarI2CXL.h"
#include "AP_RangeFinder_MaxsonarSerialLV.h"
#include "AP_RangeFinder_BBB_PRU.h"
#include "AP_RangeFinder_LightWareI2C.h"
#include "AP_RangeFinder_LightWareSerial.h"
#if (CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP ||  \
     CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO) && \
    defined(HAVE_LIBIIO)
#include "AP_RangeFinder_Bebop.h"
#endif
#include "AP_RangeFinder_MAVLink.h"
#include "AP_RangeFinder_LeddarOne.h"
#include "AP_RangeFinder_USD1_Serial.h"
#include "AP_RangeFinder_TeraRangerI2C.h"
#include "AP_RangeFinder_VL53L0X.h"
#include "AP_RangeFinder_VL53L1X.h"
#include "AP_RangeFinder_NMEA.h"
#include "AP_RangeFinder_Wasp.h"
#include "AP_RangeFinder_Benewake_TF02.h"
#include "AP_RangeFinder_Benewake_TF03.h"
#include "AP_RangeFinder_Benewake_TFMini.h"
#include "AP_RangeFinder_Benewake_TFMiniPlus.h"
#include "AP_RangeFinder_PWM.h"
#include "AP_RangeFinder_GYUS42v2.h"
#include "AP_RangeFinder_HC_SR04.h"
#include "AP_RangeFinder_BLPing.h"
#include "AP_RangeFinder_UAVCAN.h"
#include "AP_RangeFinder_Lanbao.h"
#include "AP_RangeFinder_LeddarVu8.h"
#include "AP_RangeFinder_SITL.h"
#include "AP_RangeFinder_MSP.h"
#include "AP_RangeFinder_USD1_CAN.h"
#include "AP_RangeFinder_Benewake_CAN.h"

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL &hal;

#ifndef HAL_ENCODER_MT6701_I2C_BUS
#define HAL_ENCODER_MT6701_I2C_BUS 2
#endif
#define SlaveAddress 0X0C // MT6701 地址

// table of user settable parameters
const AP_Param::GroupInfo RangeFinder::var_info[] = {

    // @Group: 1_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[0], "1_", 25, RangeFinder, AP_RangeFinder_Params),

    // @Group: 1_
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Benewake_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[0], "1_", 57, RangeFinder, backend_var_info[0]),

#if RANGEFINDER_MAX_INSTANCES > 1
    // @Group: 2_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[1], "2_", 27, RangeFinder, AP_RangeFinder_Params),

    // @Group: 2_
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Benewake_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[1], "2_", 58, RangeFinder, backend_var_info[1]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 2
    // @Group: 3_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[2], "3_", 29, RangeFinder, AP_RangeFinder_Params),

    // @Group: 3_
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Benewake_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[2], "3_", 59, RangeFinder, backend_var_info[2]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 3
    // @Group: 4_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[3], "4_", 31, RangeFinder, AP_RangeFinder_Params),

    // @Group: 4_
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Benewake_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[3], "4_", 60, RangeFinder, backend_var_info[3]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 4
    // @Group: 5_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[4], "5_", 33, RangeFinder, AP_RangeFinder_Params),

    // @Group: 5_
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Benewake_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[4], "5_", 34, RangeFinder, backend_var_info[4]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 5
    // @Group: 6_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[5], "6_", 35, RangeFinder, AP_RangeFinder_Params),

    // @Group: 6_
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Benewake_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[5], "6_", 36, RangeFinder, backend_var_info[5]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 6
    // @Group: 7_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[6], "7_", 37, RangeFinder, AP_RangeFinder_Params),

    // @Group: 7_
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Benewake_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[6], "7_", 38, RangeFinder, backend_var_info[6]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 7
    // @Group: 8_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[7], "8_", 39, RangeFinder, AP_RangeFinder_Params),

    // @Group: 8_
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Benewake_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[7], "8_", 40, RangeFinder, backend_var_info[7]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 8
    // @Group: 9_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[8], "9_", 41, RangeFinder, AP_RangeFinder_Params),

    // @Group: 9_
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Benewake_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[8], "9_", 42, RangeFinder, backend_var_info[8]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 9
    // @Group: A_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[9], "A_", 43, RangeFinder, AP_RangeFinder_Params),

    // @Group: A_
    // @Path: AP_RangeFinder_Wasp.cpp,AP_RangeFinder_Benewake_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[9], "A_", 44, RangeFinder, backend_var_info[9]),
#endif

    AP_GROUPEND};

const AP_Param::GroupInfo *RangeFinder::backend_var_info[RANGEFINDER_MAX_INSTANCES];

RangeFinder::RangeFinder()
{
    AP_Param::setup_object_defaults(this, var_info);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr)
    {
        AP_HAL::panic("Rangefinder must be singleton");
    }
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _singleton = this;
}

void RangeFinder::convert_params(void)
{
    if (params[0].type.configured_in_storage())
    {
        // _params[0]._type will always be configured in storage after conversion is done the first time
        // _params[0].type 这个变量在第一次完成转换后，在内存中将始终是被配置过的状态
        return;
    }

    struct ConversionTable
    {
        uint8_t old_element;
        uint8_t new_index;
        uint8_t instance;
    };

    // 下面定义的参数和 AP_RangeFinder_Params 类中定义的参数大部分是相同的
    const struct ConversionTable conversionTable[] = {
        // rangefinder 1
        {0, 0, 0},   // 0, TYPE 1
        {1, 1, 0},   // 1, PIN 1
        {2, 2, 0},   // 2, SCALING 1
        {3, 3, 0},   // 3, OFFSET 1
        {4, 4, 0},   // 4, FUNCTION 1
        {5, 5, 0},   // 5, MIN_CM 1
        {6, 6, 0},   // 6, MAX_CM 1
        {7, 7, 0},   // 7, STOP_PIN 1
        {9, 8, 0},   // 9, RMETRIC 1
        {10, 9, 0},  // 10, PWRRNG 1 (previously existed only once for all sensors)
        {11, 10, 0}, // 11, GNDCLEAR 1
        {23, 11, 0}, // 23, ADDR 1
        {49, 12, 0}, // 49, POS 1
        {53, 13, 0}, // 53, ORIENT 1

        // rangefinder 2
        {12, 0, 1},  // 12, TYPE 2
        {13, 1, 1},  // 13, PIN 2
        {14, 2, 1},  // 14, SCALING 2
        {15, 3, 1},  // 15, OFFSET 2
        {16, 4, 1},  // 16, FUNCTION 2
        {17, 5, 1},  // 17, MIN_CM 2
        {18, 6, 1},  // 18, MAX_CM 2
        {19, 7, 1},  // 19, STOP_PIN 2
        {21, 8, 1},  // 21, RMETRIC 2
        {10, 9, 1},  // 10, PWRRNG 1 (previously existed only once for all sensors)
        {22, 10, 1}, // 22, GNDCLEAR 2
        {24, 11, 1}, // 24, ADDR 2
        {50, 12, 1}, // 50, POS 2
        {54, 13, 1}, // 54, ORIENT 2
    };

    char param_name[17] = {0};
    AP_Param::ConversionInfo info;
    info.new_name = param_name;

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    info.old_key = 71;
#elif APM_BUILD_COPTER_OR_HELI
    info.old_key = 53;
#elif APM_BUILD_TYPE(APM_BUILD_ArduSub)
    info.old_key = 35;
#elif APM_BUILD_TYPE(APM_BUILD_Rover)
    info.old_key = 197;
#else
    // no conversion is supported on this platform
    // 此平台不支持任何转换
    params[0].type.save(true);
    return;
#endif

    for (uint8_t i = 0; i < ARRAY_SIZE(conversionTable); i++)
    {
        uint8_t param_instance = conversionTable[i].instance + 1;
        uint8_t destination_index = conversionTable[i].new_index;

        info.old_group_element = conversionTable[i].old_element;
        info.type = (ap_var_type)AP_RangeFinder_Params::var_info[destination_index].type;

        hal.util->snprintf(param_name, sizeof(param_name), "RNGFND%X_%s", param_instance, AP_RangeFinder_Params::var_info[destination_index].name);
        param_name[sizeof(param_name) - 1] = '\0';

        AP_Param::convert_old_parameter(&info, 1.0f, 0);
    }

    // force _params[0]._type into storage to flag that conversion has been done
    params[0].type.save(true);
}

/*
  initialise the RangeFinder class. We do detection of attached range
  finders here. For now we won't allow for hot-plugging of
  rangefinders.
 */
void RangeFinder::init(enum Rotation orientation_default)
{
    if (init_done)
    {
        // init called a 2nd time?
        return;
    }
    init_done = true;

    // 配置设备参数
    convert_params();

    // set orientation defaults
    // 设置方向默认值
    for (uint8_t i = 0; i < RANGEFINDER_MAX_INSTANCES; i++)
    {
        params[i].orientation.set_default(orientation_default);
    }

    // 这段代码是一个循环，用于初始化并检测一系列测距仪（rangefinder）实例。
    // 这是一个for循环，其中i用于迭代测距仪的实例，serial_instance用于追踪已加载的串行驱动实例数量。
    // 循环将继续进行，直到i达到RANGEFINDER_MAX_INSTANCES，这是允许的最大测距仪实例数。
    for (uint8_t i = 0, serial_instance = 0; i < RANGEFINDER_MAX_INSTANCES; i++)
    {
        // serial_instance will be increased inside detect_instance
        // if a serial driver is loaded for this instance
        // 如果在 detect_instance() 函数内部为这个实例加载了串行驱动, serial_instance 变量的值将增加。

        // 这行代码使用了WITH_SEMAPHORE宏（或函数），可能是为了确保线程安全或互斥访问某种资源。
        //  这里，它可能是用来保护 detect_instance() 函数的执行，确保在同一时间只有一个线程可以执行该函数。
        WITH_SEMAPHORE(detect_sem);

        // 调用detect_instance函数，传递当前的实例编号i和serial_instance作为参数。
        // 这个函数可能会尝试检测并加载该实例的测距仪驱动。
        // detect_instance函数的作用就是针对不同的传感器，调用相应的子类
        detect_instance(i, serial_instance);

        // 检查drivers数组的第i个元素是否不为空。如果不为空，说明该实例的驱动已经成功加载。
        if (drivers[i] != nullptr)
        {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy). We use MAX()
            // here as a UAVCAN rangefinder may already have been
            // found
            // 这是一系列注释，解释了如果为这个实例加载了驱动，那么该实例必须存在（尽管它可能不处于正常状态）。
            // 这里使用MAX()函数是因为可能已经找到了一个UAVCAN测距仪。

            // 更新num_instances的值，取其当前值和i + 1中的较大值。
            // 这确保了num_instances总是反映了已加载的测距仪实例的最大数量。
            num_instances = MAX(num_instances, i + 1);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "[%d] num_instances: %d\n", i, num_instances);
        }
        else
        {
            // 如果drivers[i]为空，执行以下代码块。
            gcs().send_text(MAV_SEVERITY_CRITICAL, "[%d] drivers[i] == nullptr\n", i);
        }

        // initialise status
        // 这是一个注释，说明接下来的代码将初始化状态。

        // 将第i个实例的状态设置为“NotConnected”（未连接）。
        state[i].status = Status::NotConnected;

        // 将第i个实例的有效距离计数设置为0。这可能是一个用于跟踪测距仪返回的有效距离读数的计数器。
        state[i].range_valid_count = 0;
    }
}

/*
  update RangeFinder state for all instances. This should be called at
  around 10Hz by main loop
 */
void RangeFinder::update(void)
{
    for (uint8_t i = 0; i < num_instances; i++)
    {
        if (drivers[i] != nullptr)
        {
            if ((Type)params[i].type.get() == Type::NONE)
            {
                // allow user to disable a rangefinder at runtime
                state[i].status = Status::NotConnected;
                state[i].range_valid_count = 0;
                continue;
            }
            drivers[i]->update();
        }
    }
#if HAL_LOGGING_ENABLED
    Log_RFND();
#endif
}

void RangeFinder::update_encoder(void)
{
    gcs().send_text(MAV_SEVERITY_CRITICAL, "[4-1] run RangeFinder::update_encoder() start.");
    for (uint8_t i = 0; i < num_instances; i++)
    {
        if (drivers[i] != nullptr)
        {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "[4-2] drivers[i] != nullptr.");
            if ((Type)params[i].type.get() == Type::NONE)
            {
                // allow user to disable a rangefinder at runtime
                state[i].status = Status::NotConnected;
                state[i].range_valid_count = 0;
                // 执行到此处时，因为没有检测到IIC设备连接，所以直接跳出循环了
                // 后面的drivers[i]->update_encoder()也不被执行
                gcs().send_text(MAV_SEVERITY_CRITICAL, "[4-3] (Type)params[i].type.get() == Type::NONE.");
                continue;
            }
            gcs().send_text(MAV_SEVERITY_CRITICAL, "[4-4] run drivers[i]->update_encoder() start.");

            // update函数中会调用update函数对传感器数据进行更新，update也是一个接口，
            // TeraRanger传感器继承自AP_RangeFinder_Backend_Serial，
            // 其对应的update函数在AP_RangeFinder_Backend_Serial.cpp中实现
            // 【★】update() 函数是在 AP_RangeFinder_Backend_Serial.cpp 中实现的
            // 因此考虑此处调用的函数也应该修改为与AP_RangeFinder_Backend_Serial.cpp 中的实现一致
            drivers[i]->update_encoder();
            gcs().send_text(MAV_SEVERITY_CRITICAL, "[4-5] run drivers[i]->update_encoder() finished.");
        }
    }
    gcs().send_text(MAV_SEVERITY_CRITICAL, "[4-6] run RangeFinder::update_encoder() finished.");
}

bool RangeFinder::_add_backend(AP_RangeFinder_Backend *backend, uint8_t instance, uint8_t serial_instance)
{
    if (!backend)
    {
        return false;
    }
    if (instance >= RANGEFINDER_MAX_INSTANCES)
    {
        AP_HAL::panic("Too many RANGERS backends");
    }
    if (drivers[instance] != nullptr)
    {
        // we've allocated the same instance twice
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }
    backend->init_serial(serial_instance);
    drivers[instance] = backend;
    num_instances = MAX(num_instances, instance + 1);

    return true;
}

/*
  detect if an instance of a rangefinder is connected.
  检测测距仪实例是否已连接
 */
void RangeFinder::detect_instance(uint8_t instance, uint8_t &serial_instance)
{
    const Type _type = (Type)params[instance].type.get();
    switch (_type)
    {
    case Type::PLI2C:
    case Type::PLI2CV3:
    case Type::PLI2CV3HP:
        FOREACH_I2C(i)
        {
            if (_add_backend(AP_RangeFinder_PulsedLightLRF::detect(i, state[instance], params[instance], _type),
                             instance))
            {
                break;
            }
        }
        break;
    case Type::MBI2C:
    {
        uint8_t addr = AP_RANGE_FINDER_MAXSONARI2CXL_DEFAULT_ADDR;
        if (params[instance].address != 0)
        {
            addr = params[instance].address;
        }
        FOREACH_I2C(i)
        {
            if (_add_backend(AP_RangeFinder_MaxsonarI2CXL::detect(state[instance], params[instance],
                                                                  hal.i2c_mgr->get_device(i, addr)),
                             instance))
            {
                break;
            }
        }
        break;
    }
    case Type::LWI2C:
        if (params[instance].address)
        {
            // the LW20 needs a long time to boot up, so we delay 1.5s here
            if (!hal.util->was_watchdog_armed())
            {
                hal.scheduler->delay(1500);
            }
#ifdef HAL_RANGEFINDER_LIGHTWARE_I2C_BUS
            _add_backend(AP_RangeFinder_LightWareI2C::detect(state[instance], params[instance],
                                                             hal.i2c_mgr->get_device(HAL_RANGEFINDER_LIGHTWARE_I2C_BUS, params[instance].address)),
                         instance);
#else
            FOREACH_I2C(i)
            {
                if (_add_backend(AP_RangeFinder_LightWareI2C::detect(state[instance], params[instance],
                                                                     hal.i2c_mgr->get_device(i, params[instance].address)),
                                 instance))
                {
                    break;
                }
            }
#endif
        }
        break;
    case Type::TRI2C:
        if (params[instance].address)
        {
            FOREACH_I2C(i)
            {
                if (_add_backend(AP_RangeFinder_TeraRangerI2C::detect(state[instance], params[instance],
                                                                      hal.i2c_mgr->get_device(i, params[instance].address)),
                                 instance))
                {
                    break;
                }
            }
        }
        break;
    case Type::VL53L0X:
    case Type::VL53L1X_Short:
        FOREACH_I2C(i)
        {
            if (_add_backend(AP_RangeFinder_VL53L0X::detect(state[instance], params[instance],
                                                            hal.i2c_mgr->get_device(i, params[instance].address)),
                             instance))
            {
                break;
            }
            if (_add_backend(AP_RangeFinder_VL53L1X::detect(state[instance], params[instance],
                                                            hal.i2c_mgr->get_device(i, params[instance].address),
                                                            _type == Type::VL53L1X_Short ? AP_RangeFinder_VL53L1X::DistanceMode::Short : AP_RangeFinder_VL53L1X::DistanceMode::Long),
                             instance))
            {
                break;
            }
        }
        break;
    case Type::BenewakeTFminiPlus:
    {
        uint8_t addr = TFMINI_ADDR_DEFAULT;
        if (params[instance].address != 0)
        {
            addr = params[instance].address;
        }
        FOREACH_I2C(i)
        {
            if (_add_backend(AP_RangeFinder_Benewake_TFMiniPlus::detect(state[instance], params[instance],
                                                                        hal.i2c_mgr->get_device(i, addr)),
                             instance))
            {
                break;
            }
        }
        break;
    }
    case Type::PX4_PWM:
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#ifndef HAL_BUILD_AP_PERIPH
        // to ease moving from PX4 to ChibiOS we'll lie a little about
        // the backend driver...
        if (AP_RangeFinder_PWM::detect())
        {
            _add_backend(new AP_RangeFinder_PWM(state[instance], params[instance], estimated_terrain_height), instance);
        }
#endif
#endif
        break;
    case Type::BBB_PRU:
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
        if (AP_RangeFinder_BBB_PRU::detect())
        {
            _add_backend(new AP_RangeFinder_BBB_PRU(state[instance], params[instance]), instance);
        }
#endif
        break;
    case Type::LWSER:
        if (AP_RangeFinder_LightWareSerial::detect(serial_instance))
        {
            _add_backend(new AP_RangeFinder_LightWareSerial(state[instance], params[instance]), instance, serial_instance++);
        }
        break;
    case Type::LEDDARONE:
        if (AP_RangeFinder_LeddarOne::detect(serial_instance))
        {
            _add_backend(new AP_RangeFinder_LeddarOne(state[instance], params[instance]), instance, serial_instance++);
        }
        break;
    case Type::USD1_Serial:
        if (AP_RangeFinder_USD1_Serial::detect(serial_instance))
        {
            _add_backend(new AP_RangeFinder_USD1_Serial(state[instance], params[instance]), instance, serial_instance++);
        }
        break;
    case Type::BEBOP:
#if (CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP ||  \
     CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO) && \
    defined(HAVE_LIBIIO)
        if (AP_RangeFinder_Bebop::detect())
        {
            _add_backend(new AP_RangeFinder_Bebop(state[instance], params[instance]), instance);
        }
#endif
        break;
    case Type::MAVLink:
#ifndef HAL_BUILD_AP_PERIPH
        if (AP_RangeFinder_MAVLink::detect())
        {
            _add_backend(new AP_RangeFinder_MAVLink(state[instance], params[instance]), instance);
        }
#endif
        break;
    case Type::MBSER:
        if (AP_RangeFinder_MaxsonarSerialLV::detect(serial_instance))
        {
            _add_backend(new AP_RangeFinder_MaxsonarSerialLV(state[instance], params[instance]), instance, serial_instance++);
        }
        break;
    case Type::ANALOG:
#ifndef HAL_BUILD_AP_PERIPH
        // note that analog will always come back as present if the pin is valid
        if (AP_RangeFinder_analog::detect(params[instance]))
        {
            _add_backend(new AP_RangeFinder_analog(state[instance], params[instance]), instance);
        }
#endif
        break;
    case Type::HC_SR04:
#ifndef HAL_BUILD_AP_PERIPH
        // note that this will always come back as present if the pin is valid
        if (AP_RangeFinder_HC_SR04::detect(params[instance]))
        {
            _add_backend(new AP_RangeFinder_HC_SR04(state[instance], params[instance]), instance);
        }
#endif
        break;
    case Type::NMEA:
        if (AP_RangeFinder_NMEA::detect(serial_instance))
        {
            _add_backend(new AP_RangeFinder_NMEA(state[instance], params[instance]), instance, serial_instance++);
        }
        break;
    case Type::WASP:
        if (AP_RangeFinder_Wasp::detect(serial_instance))
        {
            _add_backend(new AP_RangeFinder_Wasp(state[instance], params[instance]), instance, serial_instance++);
        }
        break;
    case Type::BenewakeTF02:
        if (AP_RangeFinder_Benewake_TF02::detect(serial_instance))
        {
            _add_backend(new AP_RangeFinder_Benewake_TF02(state[instance], params[instance]), instance, serial_instance++);
        }
        break;
    case Type::BenewakeTFmini:
        if (AP_RangeFinder_Benewake_TFMini::detect(serial_instance))
        {
            _add_backend(new AP_RangeFinder_Benewake_TFMini(state[instance], params[instance]), instance, serial_instance++);
        }
        break;
    case Type::BenewakeTF03:
        if (AP_RangeFinder_Benewake_TF03::detect(serial_instance))
        {
            _add_backend(new AP_RangeFinder_Benewake_TF03(state[instance], params[instance]), instance, serial_instance++);
        }
        break;
    case Type::PWM:
#ifndef HAL_BUILD_AP_PERIPH
        if (AP_RangeFinder_PWM::detect())
        {
            _add_backend(new AP_RangeFinder_PWM(state[instance], params[instance], estimated_terrain_height), instance);
        }
#endif
        break;
    case Type::BLPing:
        if (AP_RangeFinder_BLPing::detect(serial_instance))
        {
            _add_backend(new AP_RangeFinder_BLPing(state[instance], params[instance]), instance, serial_instance++);
        }
        break;
    case Type::Lanbao:
        if (AP_RangeFinder_Lanbao::detect(serial_instance))
        {
            _add_backend(new AP_RangeFinder_Lanbao(state[instance], params[instance]), instance, serial_instance++);
        }
        break;
    case Type::LeddarVu8_Serial:
        if (AP_RangeFinder_LeddarVu8::detect(serial_instance))
        {
            _add_backend(new AP_RangeFinder_LeddarVu8(state[instance], params[instance]), instance, serial_instance++);
        }
        break;

    case Type::UAVCAN:
#if HAL_CANMANAGER_ENABLED
        /*
          the UAVCAN driver gets created when we first receive a
          measurement. We take the instance slot now, even if we don't
          yet have the driver
         */
        num_instances = MAX(num_instances, instance + 1);
#endif
        break;

    case Type::GYUS42v2:
        if (AP_RangeFinder_GYUS42v2::detect(serial_instance))
        {
            _add_backend(new AP_RangeFinder_GYUS42v2(state[instance], params[instance]), instance, serial_instance++);
        }
        break;

    case Type::SIM:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        _add_backend(new AP_RangeFinder_SITL(state[instance], params[instance], instance), instance);
#endif
        break;

    case Type::MSP:
#if HAL_MSP_RANGEFINDER_ENABLED
        if (AP_RangeFinder_MSP::detect())
        {
            _add_backend(new AP_RangeFinder_MSP(state[instance], params[instance]), instance);
        }
#endif // HAL_MSP_RANGEFINDER_ENABLED
        break;

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    case Type::USD1_CAN:
        _add_backend(new AP_RangeFinder_USD1_CAN(state[instance], params[instance]), instance);
        break;
    case Type::Benewake_CAN:
        _add_backend(new AP_RangeFinder_Benewake_CAN(state[instance], params[instance]), instance);
        break;
#endif
    case Type::NONE:
    default:
        break;
    }

    // #ifdef HAL_ENCODER_MT6701_I2C_BUS
    /*
     * bus_clock=40 0000
     */

    // 在此处加入延时函数，看看延时过后能不能在地面站收到输出信息
    // 有效果，但delay(1000)时只能看到从3号开始的设备
    // 0，1，2号设备看不到
    // delay(2000)时可以看到从0号设备开始的初始化情况
    hal.scheduler->delay(3000);

    // gcs().send_text(MAV_SEVERITY_CRITICAL, "[1-1] _add_backend start.");
    // if (_add_backend(AP_RangeFinder_LightWareI2C::detect(state[instance], params[instance],
    //                                                      hal.i2c_mgr->get_device(HAL_ENCODER_MT6701_I2C_BUS, SlaveAddress)),
    //                  instance))
    // {
    //     gcs().send_text(MAV_SEVERITY_CRITICAL, "[1-2] _add_backend successed.");
    // }
    // else
    // {
    //     gcs().send_text(MAV_SEVERITY_CRITICAL, "[1-3] _add_backend failed.");
    // }
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "[1-4] state[instance]: %d.", (int)state[instance].status);
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "[1-5] _add_backend finish.");

    /*
     * Fast Mode:          hi2c1.Init.Timing = 0x0010061A;  十进制：105 0138
     * Standerd Mode:      hi2c1.Init.Timing = 0x00303D5B;  十进制：316 1435
     */

    gcs().send_text(MAV_SEVERITY_CRITICAL, "[1-1] _add_backend start.");

    // 调用_add_backend函数将接口放到一个指针数组中，方便通过数组轮流调用相应的接口
    // _add_backend 这个函数就是把上面查找到的传感器接口放入指针数组drivers中，在update中调用
    // 其中 AP_RangeFinder_LightWareI2C::detect 函数是一个静态成员函数调用，属于 AP_RangeFinder_LightWareI2C 类。
    // 它用于检测LightWare I2C接口的测距仪是否存在并可以正常工作。函数接受三个参数：
    // state[instance]：这是一个数组，包含了每个测距仪实例的状态信息。instance 是当前正在检测的实例的索引。
    // params[instance]：这是一个包含每个测距仪实例配置参数（传感器类型、引脚、地址、安装方向等）的数组，与IIC通信关系不大。
    // hal.i2c_mgr->get_device(HAL_ENCODER_MT6701_I2C_BUS, SlaveAddress)：
    // 这调用了一个硬件抽象层（HAL）的I2C管理器函数，用于获取指定I2C总线（HAL_ENCODER_MT6701_I2C_BUS）上的从设备（SlaveAddress）。
    // 这个 if 语句检查 _add_backend 函数的返回值。如果 _add_backend 成功添加了一个后端（即返回值为 true），则执行 if 语句块内的代码。
    // 总结一下，这段代码的主要作用是尝试使用LightWare I2C接口检测并添加一个测距仪后端。如果成功添加了后端，它将向地面控制站发送一条确认消息。

    // 这段代码直接使用了HAL_ENCODER_MT6701_I2C_BUS宏定义的I2C总线编号来获取设备，并尝试添加后端。
    // 这里假定LightWare测距仪总是连接在特定的I2C总线上。
    // if(_add_backend(AP_RangeFinder_LightWareI2C::detect(state[instance], params[instance],
    //                                                  hal.i2c_mgr->get_device(HAL_ENCODER_MT6701_I2C_BUS, SlaveAddress)),
    //                                                  instance)){
    //                                                     gcs().send_text(MAV_SEVERITY_CRITICAL, "[1-2] _add_backend successed.");
    //                                                  }
    // else{
    //     gcs().send_text(MAV_SEVERITY_CRITICAL, "[1-3] _add_backend failed.");
    // }

    // 在没有定义HAL_RANGEFINDER_LIGHTWARE_I2C_BUS的情况下，
    // 代码将遍历所有可用的I2C总线，尝试在每个总线上找到LightWare测距仪（MT6701）并添加后端。
    FOREACH_I2C(i)
    {
        if (_add_backend(AP_RangeFinder_LightWareI2C::detect(state[instance], params[instance],
                                                             hal.i2c_mgr->get_device(i, SlaveAddress)),
                         instance)){
                            gcs().send_text(MAV_SEVERITY_CRITICAL, "[1-2] _add_backend successed[%ld].", i);
                            break;
                         }
        else{
            gcs().send_text(MAV_SEVERITY_CRITICAL, "[1-3] _add_backend failed.");
        }
    }
    gcs().send_text(MAV_SEVERITY_CRITICAL, "[1-4] state[instance]: %d.", (int)state[instance].status);
    gcs().send_text(MAV_SEVERITY_CRITICAL, "[1-5] _add_backend finish.");

    // #else
    //             gcs().send_text(MAV_SEVERITY_CRITICAL, "[1-2] HAL_ENCODER_MT6701_I2C_BUS not defined.");
    //             FOREACH_I2C(i) {
    //                 if (_add_backend(AP_RangeFinder_LightWareI2C::detect(state[instance], params[instance],
    //                                                                      hal.i2c_mgr->get_device(i, params[instance].address)),
    //                                  instance)) {
    //                     gcs().send_text(MAV_SEVERITY_CRITICAL, "[1-3] _add_backend successed.");
    //                     break;
    //                 }
    //             }
    //             gcs().send_text(MAV_SEVERITY_CRITICAL, "[1-4] _add_backend finish.");
    // #endif

    // if the backend has some local parameters then make those available in the tree
    if (drivers[instance] && state[instance].var_info)
    {
        backend_var_info[instance] = state[instance].var_info;
        AP_Param::load_object_from_eeprom(drivers[instance], backend_var_info[instance]);

        // param count could have changed
        AP_Param::invalidate_count();
    }
}

AP_RangeFinder_Backend *RangeFinder::get_backend(uint8_t id) const
{
    if (id >= num_instances)
    {
        return nullptr;
    }
    if (drivers[id] != nullptr)
    {
        if (drivers[id]->type() == Type::NONE)
        {
            // pretend it isn't here; disabled at runtime?
            return nullptr;
        }
    }
    return drivers[id];
};

RangeFinder::Status RangeFinder::status_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr)
    {
        return Status::NotConnected;
    }
    return backend->status();
}

void RangeFinder::handle_msg(const mavlink_message_t &msg)
{
    uint8_t i;
    for (i = 0; i < num_instances; i++)
    {
        if ((drivers[i] != nullptr) && ((Type)params[i].type.get() != Type::NONE))
        {
            drivers[i]->handle_msg(msg);
        }
    }
}

#if HAL_MSP_RANGEFINDER_ENABLED
void RangeFinder::handle_msp(const MSP::msp_rangefinder_data_message_t &pkt)
{
    uint8_t i;
    for (i = 0; i < num_instances; i++)
    {
        if ((drivers[i] != nullptr) && ((Type)params[i].type.get() == Type::MSP))
        {
            drivers[i]->handle_msp(pkt);
        }
    }
}
#endif // HAL_MSP_RANGEFINDER_ENABLED

// return true if we have a range finder with the specified orientation
bool RangeFinder::has_orientation(enum Rotation orientation) const
{
    return (find_instance(orientation) != nullptr);
}

// find first range finder instance with the specified orientation
AP_RangeFinder_Backend *RangeFinder::find_instance(enum Rotation orientation) const
{
    // first try for a rangefinder that is in range
    for (uint8_t i = 0; i < num_instances; i++)
    {
        AP_RangeFinder_Backend *backend = get_backend(i);
        if (backend != nullptr &&
            backend->orientation() == orientation &&
            backend->status() == Status::Good)
        {
            return backend;
        }
    }
    // if none in range then return first with correct orientation
    for (uint8_t i = 0; i < num_instances; i++)
    {
        AP_RangeFinder_Backend *backend = get_backend(i);
        if (backend != nullptr &&
            backend->orientation() == orientation)
        {
            return backend;
        }
    }
    return nullptr;
}

float RangeFinder::distance_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr)
    {
        return 0;
    }
    return backend->distance();
}

uint16_t RangeFinder::distance_cm_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr)
    {
        return 0;
    }
    return backend->distance_cm();
}

int16_t RangeFinder::max_distance_cm_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr)
    {
        return 0;
    }
    return backend->max_distance_cm();
}

int16_t RangeFinder::min_distance_cm_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr)
    {
        return 0;
    }
    return backend->min_distance_cm();
}

int16_t RangeFinder::ground_clearance_cm_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr)
    {
        return 0;
    }
    return backend->ground_clearance_cm();
}

bool RangeFinder::has_data_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr)
    {
        return false;
    }
    return backend->has_data();
}

uint8_t RangeFinder::range_valid_count_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr)
    {
        return 0;
    }
    return backend->range_valid_count();
}

const Vector3f &RangeFinder::get_pos_offset_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr)
    {
        return pos_offset_zero;
    }
    return backend->get_pos_offset();
}

uint32_t RangeFinder::last_reading_ms(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr)
    {
        return 0;
    }
    return backend->last_reading_ms();
}

MAV_DISTANCE_SENSOR RangeFinder::get_mav_distance_sensor_type_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr)
    {
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }
    return backend->get_mav_distance_sensor_type();
}

// get temperature reading in C.  returns true on success and populates temp argument
bool RangeFinder::get_temp(enum Rotation orientation, float &temp) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr)
    {
        return false;
    }
    return backend->get_temp(temp);
}

// Write an RFND (rangefinder) packet
void RangeFinder::Log_RFND() const
{
    if (_log_rfnd_bit == uint32_t(-1))
    {
        return;
    }

    AP_Logger &logger = AP::logger();
    if (!logger.should_log(_log_rfnd_bit))
    {
        return;
    }

    for (uint8_t i = 0; i < RANGEFINDER_MAX_INSTANCES; i++)
    {
        const AP_RangeFinder_Backend *s = get_backend(i);
        if (s == nullptr)
        {
            continue;
        }

        const struct log_RFND pkt = {
            LOG_PACKET_HEADER_INIT(LOG_RFND_MSG),
            time_us : AP_HAL::micros64(),
            instance : i,
            dist : s->distance_cm(),
            status : (uint8_t)s->status(),
            orient : s->orientation(),
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }
}

bool RangeFinder::prearm_healthy(char *failure_msg, const uint8_t failure_msg_len) const
{
    for (uint8_t i = 0; i < RANGEFINDER_MAX_INSTANCES; i++)
    {
        if ((Type)params[i].type.get() == Type::NONE)
        {
            continue;
        }

        if (drivers[i] == nullptr)
        {
            hal.util->snprintf(failure_msg, failure_msg_len, "Rangefinder %X: Not Detected", i + 1);
            return false;
        }

        switch (drivers[i]->status())
        {
        case Status::NoData:
            hal.util->snprintf(failure_msg, failure_msg_len, "Rangefinder %X: No Data", i + 1);
            return false;
        case Status::NotConnected:
            hal.util->snprintf(failure_msg, failure_msg_len, "Rangefinder %X: Not Connected", i + 1);
            return false;
        case Status::OutOfRangeLow:
        case Status::OutOfRangeHigh:
        case Status::Good:
            break;
        }
    }

    return true;
}

RangeFinder *RangeFinder::_singleton;

namespace AP
{

    RangeFinder *rangefinder()
    {
        return RangeFinder::get_singleton();
    }

}
