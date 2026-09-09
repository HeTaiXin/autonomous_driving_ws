#ifndef PNC_MAP_CREATOR_STURN_H_
#define PNC_MAP_CREATOR_STURN_H_

#include "pnc_map_creator_base.h"

namespace Planning
{
    class PNCMapCreatorSTurn : public PNCMapCreatorBase // S弯道地图
    {
    public:
        PNCMapCreatorSTurn();

        // 重写基类的虚构函数，完成PNCMapCreatorBase对象的销毁工作。
        ~PNCMapCreatorSTurn() override = default;
        // 履行对creat_pnc_map接口的承诺，提供具体的函数内容（生成地图）
        void creat_pnc_map() override;

    private:
        // 初始化地图
        void init_pnc_map();

        // 沿x轴画直线
        void draw_straight_x(const double &length, const double &plus_flag, const double &ratio = 1.0);

        // 画弧线，正方向为逆时针，负方向为顺时针，angle为总角度
        void draw_arc(const double &angle, const double &plus_flag, const double &ratio = 1.0);
    };
} // namespace Planning

#endif // PNC_MAP_CREATOR_STURN_H_