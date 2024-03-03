#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class AP_RangeFinder_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_RangeFinder_Params(void);

    /* Do not allow copies */
    /*
     * 这两行代码是C++中的特殊声明，用于明确禁止 "类的拷贝构造函数" 和 "拷贝赋值运算符" 的使用。
     * AP_RangeFinder_Params(const AP_RangeFinder_Params &other) = delete;
     * 这行代码是AP_RangeFinder_Params "类的拷贝构造函数" 的声明，并且它被设置为delete，
     * 意味着这个 "拷贝构造函数" 在类中是不可用的。这意味着你不能使用以下语法来创建一个AP_RangeFinder_Params对象的副本：
     * AP_RangeFinder_Params original;  
     * AP_RangeFinder_Params copy(original); // 这会导致编译错误
     * 
     * AP_RangeFinder_Params &operator=(const AP_RangeFinder_Params&) = delete;
     * 这行代码是AP_RangeFinder_Params类的拷贝赋值运算符的声明，同样被设置为delete。
     * 这意味着你不能使用以下语法来将一个AP_RangeFinder_Params对象赋值给另一个同类型的对象：
     * AP_RangeFinder_Params original;  
     * AP_RangeFinder_Params another;  
     * another = original; // 这会导致编译错误
     * 禁止 "拷贝构造函数" 和 "拷贝赋值运算符" 通常是为了防止对象被不正确地复制，
     * 特别是当对象拥有一些不应该被复制的资源（如文件句柄、网络连接、动态分配的内存等）时。
     * 在这些情况下，通常希望对象能够通过移动语义（移动构造函数和移动赋值运算符）来转移资源所有权，而不是通过复制。
     * 在某些设计中，禁止拷贝而只允许移动也是为了实现对象的唯一性，确保在程序中始终只有一个实例（例如单例模式）。
     * 
     * 【★】如果AP_RangeFinder_Params类是一个包含配置或参数的结构体或类，并且这些参数不应该被复制，那么禁止拷贝操作是有意义的。
     * 【★】例如，如果这个类的实例代表了一些硬件设备的配置，那么复制这个实例可能会导致两个设备尝试使用相同的配置，这通常是不允许的。
     *
     */
    AP_RangeFinder_Params(const AP_RangeFinder_Params &other) = delete;
    AP_RangeFinder_Params &operator=(const AP_RangeFinder_Params&) = delete;

    AP_Vector3f pos_offset; // position offset in body frame
    AP_Float scaling;
    AP_Float offset;
    AP_Int16 powersave_range;
    AP_Int16 min_distance_cm;
    AP_Int16 max_distance_cm;
    AP_Int8  type;
    AP_Int8  pin;
    AP_Int8  ratiometric;
    AP_Int8  stop_pin;
    AP_Int8  function;
    AP_Int8  ground_clearance_cm;
    AP_Int8  address;
    AP_Int8  orientation;
};
