#include "mode.h"
#include "Plane.h"

/*
 * ADS-B (Automatic Dependent Surveillance-Broadcast) 是自动相关监视广播的简称，
 * ADS-B是一种基于全球卫星定位系统和利用空地、空空数据链实现交通监控和信息传递的空管监视新技术。
 * 系统无需人工操作或者询问，可以自动地（每一秒一次）从相关机载设备获取参数，
 * 向其他飞机或地面站广播飞机的位置、高度、速度、航向、识别号等信息，以供管制员对飞机状态进行监控。
 * ADS-B不仅可以安装在民航飞机上还可以安装在无人机上，解决飞行器之间的相互碰撞问题
 * */

#if HAL_ADSB_ENABLED

bool ModeAvoidADSB::_enter()
{
    return plane.mode_guided.enter();
}

void ModeAvoidADSB::update()
{
    plane.mode_guided.update();
}

void ModeAvoidADSB::navigate()
{
    // Zero indicates to use WP_LOITER_RAD
    plane.update_loiter(0);
}

#endif // HAL_ADSB_ENABLED

