#ifndef PNC_MAP_CREATOR_STRAIGHT_H_
#define PNC_MAP_CREATOR_STRAIGHT_H_

#include "pnc_map_creator_base.h"

namespace Planning
{
    class PNCMapCreatorStraight : public PNCMapCreatorBase // 直道地图
    {
    public:
        PNCMapCreatorStraight();
        
        // 重写基类的虚构函数，完成PNCMapCreatorBase对象的销毁工作。
        ~PNCMapCreatorStraight() override = default;
        // 履行对creat_pnc_map接口的承诺，提供具体的函数内容（生成地图）
        void creat_pnc_map() override;

    private:
        void init_pnc_map(); // 初始化地图
        void draw_straight_x(const double &length, const double &plus_flag, const double &ratio = 1.0); // 沿x轴画直线
    };
} // namespace Planning

#endif // PNC_MAP_CREATOR_STRAIGHT_H_