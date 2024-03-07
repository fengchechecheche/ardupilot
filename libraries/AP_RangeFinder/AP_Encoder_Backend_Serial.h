#include "AP_Encoder_Backend.h"
#include "AP_Encoder.h"

extern const AP_HAL::HAL& hal;

class AP_Encoder_Backend_Serial : public AP_Encoder_Backend
{
public:
    // constructor
    AP_Encoder_Backend_Serial(AP_Encoder& encoder);

    void init_serial(uint8_t serial_instance) override;
    // static detection function
    static bool detect(uint8_t serial_instance);

protected:

    // baudrate used during object construction:
    virtual uint32_t initial_baudrate(uint8_t serial_instance) const;

    // the value 0 is special to the UARTDriver - it's "use default"
    virtual uint16_t rx_bufsize() const { return 0; }
    virtual uint16_t tx_bufsize() const { return 0; }

    AP_HAL::UARTDriver *uart = nullptr;
};
