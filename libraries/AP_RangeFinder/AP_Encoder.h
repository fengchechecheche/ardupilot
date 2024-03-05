#ifndef __AP_ENCODER_H__
#define __AP_ENCODER_H__

/*
 * 前台类对应用层负责，因此在写前台类时，应该考虑如何能让应用层方便调用。
 * C++语言在写类时，几个基本的要素(搭建框架)如下：
 * 1.先写类的构造函数和析构函数。
 * 
 * 
 */

// 在ardupilot的前后台架构中，前台类里面一般都会有后台类的前置声明
// 因为这里使用了[前置声明]，所以就不再需要在AP_Encoder.h头文件中包含后台类的头文件
// 同时会有一个后台类的指针_driver
class AP_Encoder_Backend;

class AP_Encoder{
    friend class AP_Encoder_Backend;
public:
    AP_Encoder();       // 类的构造函数在.cpp源文件中进行实现
    ~AP_Encoder(){};    // 此处类的析构函数为空实现

    void init(void);    // 希望应用层调用一下初始化函数，就能够把相应的硬件初始化掉
    double read(void);  // 希望能够读取到编码器中的数据，以double数据类型返回一个值
    void update(void);
private:
    // 前台类通过后台类的这个指针，就可以和后台类进行交流
    AP_Encoder_Backend* _driver;
};














#endif
