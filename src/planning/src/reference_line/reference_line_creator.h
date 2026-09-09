#ifndef REFERENCE_LINE_CREATOR_H_
#define REFERENCE_LINE_CREATOR_H_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "config_reader.h"
#include "curve.h"
#include "reference_line_smoother.h"
#include <cmath>

namespace Planning
{
    using base_msgs::msg::Referline;
    using base_msgs::msg::ReferlinePoint;
    using geometry_msgs::msg::PoseStamped;
    using nav_msgs::msg::Path;

    class ReferenceLineCreator // 创建参考线
    {
    public:
        ReferenceLineCreator();

        // 生成参考线
        Referline create_reference_line(const Path &global_path, const PoseStamped &target_point);
        
        // 转换成rviz显示的Path格式
        Path referline_to_rviz();

    public:
        inline const Referline &get_reference_line() const { return refer_line_; }      // 获取参考线
        inline const Path &get_reference_line_rviz() const { return refer_line_rviz_; } // 获取参考线 for rviz
        inline const int &get_match_point_index() const { return match_point_index_; }  // 获取匹配点下标
        inline const int &get_front_index() const { return front_index_; }              // 获取最前点下标
        inline const int &get_back_index() const { return back_index_; }                // 获取最后点下标

    private:
        std::unique_ptr<ConfigReader> reference_line_config_;        // 配置
        Referline refer_line_;                                       // 参考线
        Path refer_line_rviz_;                                       // 参考线 for rviz
        std::shared_ptr<ReferenceLineSmoother> refer_line_smoother_; // 参考线平滑器
        int last_match_point_index_{-1};                            // 上一帧匹配点在全局路径下的下标
        int match_point_index_{-1};                                 // 匹配点在全局路径下的下标
        int front_index_{-1};                                       // 最前点在全局路径下的下标
        int back_index_{-1};                                        // 最后点在全局路径下的下标
    };
} // namespace Planning

#endif // REFERENCE_LINE_CREATOR_H_