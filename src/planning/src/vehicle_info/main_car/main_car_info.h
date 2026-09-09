#ifndef MAIN_CAR_INFO_H_
#define MAIN_CAR_INFO_H_

#include "vehicle_info_base.h"

namespace Planning
{
    class MainCarInfo : public VehicleInfoBase
    {
    public:
        MainCarInfo(); // 主车

        // 定位点转frenet
        void vehicle_cartesian_to_frenet(const Referline &refer_line) override; // 定位点在参考线上的投影点参数
    };
} // namespace Planning

#endif // MAIN_CAR_INFO_H_