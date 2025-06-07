#ifndef __AP_ENCODER_H__
#define __AP_ENCODER_H__

#ifndef ENCODER_MAX_INSTANCES
#define ENCODER_MAX_INSTANCES 2
#endif

class AP_Encoder_Backend;

extern const AP_HAL::HAL &hal;
#define SlaveAddress    0X06        //MT6701 地址

class AP_Encoder{
    friend class AP_Encoder_Backend;
public:
    AP_Encoder();       // 类的构造函数在.cpp源文件中进行实现
    ~AP_Encoder(){};    // 此处类的析构函数为空实现

    void init(void);    // 希望应用层调用一下初始化函数，就能够把相应的硬件初始化掉
    void update(void);
private:
    uint8_t num_instances;
    // 前台类通过后台类的这个指针，就可以和后台类进行交流
    AP_Encoder_Backend *drivers[ENCODER_MAX_INSTANCES];
    bool _add_backend(AP_Encoder_Backend *backend, uint8_t instance, uint8_t serial_instance);
};

#endif
