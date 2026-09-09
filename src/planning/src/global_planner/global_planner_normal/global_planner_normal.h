#ifndef GLOBAL_PLANNER_NORMAL_H_
#define GLOBAL_PLANNER_NORMAL_H_

#include "global_planner_base.h"

namespace Planning
{
    class GlobalPlannerNormal : public GlobalPlannerBase // 普通全局路径规划
    {
    public:
        GlobalPlannerNormal();

        ~GlobalPlannerNormal() override = default;
        void search_global_path(const PNCMap &pnc_map) override; // 搜索全部路径
    };
} // namespace Planning

#endif // GLOBAL_PLANNER_NORMAL_H_