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

#include "AP_RangeFinder_LightWareSerial.h"

#include <AP_HAL/AP_HAL.h>
#include <ctype.h>

extern const AP_HAL::HAL& hal;

#define LIGHTWARE_DIST_MAX_CM           10000
#define LIGHTWARE_OUT_OF_RANGE_ADD_CM   100

// read - return last value measured by sensor
bool AP_RangeFinder_LightWareSerial::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    float sum = 0;              // sum of all readings taken
    uint16_t valid_count = 0;   // number of valid readings
    uint16_t invalid_count = 0; // number of invalid readings

    // max distance the sensor can reliably measure - read from parameters
    const int16_t distance_cm_max = max_distance_cm();

    // read any available lines from the lidar
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        char c = uart->read();

        // use legacy protocol
        // 这段代码是一个处理特定协议的片段，可能是从串口或其他数据流中读取数据并进行解析。
        // 1.这部分代码检查当前的协议状态。如果状态是UNKNOWN（未知）或LEGACY（旧版），则执行以下操作。
        if (protocol_state == ProtocolState::UNKNOWN || protocol_state == ProtocolState::LEGACY) {
            // 2. 回车符处理:
            // 当读取到字符c为回车符（\r）时，执行以下操作：
            if (c == '\r') {
                // 在linebuf的当前长度位置放置一个空字符（\0），使linebuf成为一个合法的C字符串。
                linebuf[linebuf_len] = 0;
                // 使用strtof函数将linebuf中的字符串转换为浮点数dist。
                const float dist = strtof(linebuf, nullptr);
                // 这部分代码检查dist是否是一个非负值，并且不是一个表示信号丢失的特定距离值。
                if (!is_negative(dist) && !is_lost_signal_distance(dist * 100, distance_cm_max)) {
                    // 累加距离和更新有效计数:
                    // 如果上述条件满足，将dist累加到sum中，并增加valid_count。
                    sum += dist;
                    valid_count++;
                    // if still determining protocol update legacy valid count
                    // 更新旧版协议的有效计数:
                    // 如果协议状态是UNKNOWN，则增加legacy_valid_count。
                    if (protocol_state == ProtocolState::UNKNOWN) {
                        legacy_valid_count++;
                    }
                } else {
                    // 增加无效计数:
                    // 如果dist不满足上述条件，则增加invalid_count。
                    invalid_count++;
                }
                // 重置行缓冲区长度:
                // 将linebuf的长度重置为0，以便为下一行数据做准备。
                linebuf_len = 0;
            } 
            // 3. 处理数字、小数点和负号:
            // 如果读取到的字符c是数字、小数点或负号，则将其添加到linebuf中，并增加linebuf_len。
            // 同时，检查linebuf是否已满（即是否达到其大小）。如果已满，则重置linebuf_len为0，丢弃当前行。
            else if (isdigit(c) || c == '.' || c == '-') {
                linebuf[linebuf_len++] = c;
                if (linebuf_len == sizeof(linebuf)) {
                    // too long, discard the line
                    linebuf_len = 0;
                }
            }
        }

        // use binary protocol
        // 这段代码是处理二进制协议的片段。它似乎是从一个数据流中读取数据，并根据二进制格式解析这些数据。
        // 1.协议状态检查：
        // 如果当前的协议状态是UNKNOWN（未知）或BINARY（二进制），则执行以下操作。
        if (protocol_state == ProtocolState::UNKNOWN || protocol_state == ProtocolState::BINARY) {
            // 2.检查最高位：
            // 使用BIT_IS_SET宏（或函数）检查字符c的最高位（第7位）是否设置。
            // 这通常用于确定数据字节是高字节还是低字节。
            bool msb_set = BIT_IS_SET(c, 7);
            // 3.处理高字节：
            // 如果最高位被设置，表示接收到的是高字节。
            // 将c的值赋给high_byte变量，并设置high_byte_received标志为true。
            if (msb_set) {
                // received the high byte
                high_byte = c;
                high_byte_received = true;
            } 
            // 4.处理低字节：
            // 如果最高位没有被设置，表示接收到的是低字节。
            // 此时，如果high_byte_received标志为true（即已经接收到了一个高字节），则执行以下操作：
            else {
                // received the low byte which should be second
                if (high_byte_received) {
                    // a. 组合高字节和低字节：
                    // 通过位操作组合高字节和低字节，创建一个16位整数dist。
                    // 这里使用& 0x7f来移除每个字节的最高位（通常用作符号位），并通过左移和按位或操作组合它们。
                    const int16_t dist = (high_byte & 0x7f) << 7 | (c & 0x7f);
                    // b. 验证距离值：
                    // 检查组合后的距离值dist是否非负，并且不是表示信号丢失的特定距离值。
                    if (dist >= 0 && !is_lost_signal_distance(dist, distance_cm_max)) {
                        // c. 更新统计信息：
                        // 如果dist满足上述条件，将其乘以0.01（可能是从毫米转换为厘米或其他单位），
                        // 并累加到sum中。同时，增加valid_count。
                        sum += dist * 0.01f;
                        valid_count++;
                        // if still determining protocol update binary valid count
                        // d. 更新二进制协议的有效计数：
                        // 如果协议状态仍然是UNKNOWN，则增加binary_valid_count。
                        if (protocol_state == ProtocolState::UNKNOWN) {
                            binary_valid_count++;
                        }
                    } 
                    // e. 增加无效计数：
                    // 如果dist不满足上述条件，则增加invalid_count。
                    else {
                        invalid_count++;
                    }
                }
                // f. 重置高字节接收标志：
                // 无论如何，都需要重置high_byte_received标志，以便为下一组高低字节做准备。
                high_byte_received = false;
            }
        }
    }

    // protocol set after 10 successful reads
    if (protocol_state == ProtocolState::UNKNOWN) {
        if (binary_valid_count > 10) {
            protocol_state = ProtocolState::BINARY;
        } else if (legacy_valid_count > 10) {
            protocol_state = ProtocolState::LEGACY;
        }
    }

    uint32_t now = AP_HAL::millis();
    if (last_init_ms == 0 ||
        (now - last_init_ms > 1000 &&
         now - state.last_reading_ms > 1000)) {
        // send enough serial transitions to trigger LW20 into serial
        // mode. It starts in dual I2C/serial mode, and wants to see
        // enough transitions to switch into serial mode.
        uart->write("www\r\n");
        last_init_ms = now;
    } else {
        uart->write('d');
    }

    // return average of all valid readings
    if (valid_count > 0) {
        reading_m = sum / valid_count;
        no_signal = false;
        return true;
    }

    // all readings were invalid so return out-of-range-high value
    if (invalid_count > 0) {
        reading_m = MIN(MAX(LIGHTWARE_DIST_MAX_CM, distance_cm_max + LIGHTWARE_OUT_OF_RANGE_ADD_CM), UINT16_MAX) * 0.01f;
        no_signal = true;
        return true;
    }

    // no readings so return false
    return false;
}

// check to see if distance returned by the LiDAR is a known lost-signal distance flag
bool AP_RangeFinder_LightWareSerial::is_lost_signal_distance(int16_t distance_cm, int16_t distance_cm_max)
{
    if (distance_cm < distance_cm_max + LIGHTWARE_OUT_OF_RANGE_ADD_CM) {
        // in-range
        return false;
    }
    const int16_t bad_distances[] { 13000, 16000, 23000, 25000 };
    for (const auto bad_distance_cm : bad_distances) {
        if (distance_cm == bad_distance_cm) {
            return true;
        }
    }
    return false;
}
