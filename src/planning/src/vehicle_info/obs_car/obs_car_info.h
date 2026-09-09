#ifndef OBS_CAR_INFO_H_
#define OBS_CAR_INFO_H_

#include "vehicle_info_base.h"

namespace Planning
{
class ObsCarInfo : public VehicleInfoBase
{
public:
    ObsCarInfo(const int &id);  // 障碍物

    // 重写基类的纯虚函数
    void vehicle_cartesian_to_frenet(const Referline &refer_line) override;
private:
};
}  // namespace Planning

#endif  // OBS_CAR_INFO_H_