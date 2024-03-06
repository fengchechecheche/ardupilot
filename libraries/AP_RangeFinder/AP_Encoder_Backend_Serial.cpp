#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include "AP_Encoder_Backend_Serial.h"

extern const AP_HAL::HAL& hal;

AP_Encoder_Backend_Serial::AP_Encoder_Backend_Serial(AP_Encoder& encoder)
: AP_Encoder_Backend(encoder)
{

}

void AP_Encoder_Backend_Serial::init_serial(uint8_t serial_instance)
{
    uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance);
    if (uart != nullptr) {
        uart->begin(initial_baudrate(serial_instance), rx_bufsize(), tx_bufsize());
    }
}

uint32_t AP_Encoder_Backend_Serial::initial_baudrate(const uint8_t serial_instance) const
{
    return AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance);
}
