#include "reference_line_creator.h"

namespace Planning
{
    ReferenceLineCreator::ReferenceLineCreator() // 创建参考线
    {
        RCLCPP_INFO(rclcpp::get_logger("reference_line"), "reference_line_creator created.");

        // 读取配置文件
        reference_line_config_ = std::make_unique<ConfigReader>();
        reference_line_config_->read_reference_line_config(); // 包含read_pnc_map_config()

        // 创建平滑器
        refer_line_smoother_ = std::make_shared<ReferenceLineSmoother>();
    }

    // 生成参考线
    Referline ReferenceLineCreator::create_reference_line(const Path &global_path, const PoseStamped &target_point)
    {
        if (global_path.poses.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger("reference_line"), "global_path is an empty set");
            return refer_line_;
        }

        // 参考线消息头
        refer_line_.header.frame_id = reference_line_config_->get_pnc_map().frame_;
        refer_line_.header.stamp = rclcpp::Clock().now();
        refer_line_.refer_line.clear(); // 清除旧数据

        /*
        1. 找匹配点:
            通过全路路径(global_path_)找到主车的匹配点下标(match_point_index_);
        */
        match_point_index_ = Curve::find_match_point(global_path, last_match_point_index_, target_point);
        last_match_point_index_ = match_point_index_; // 更新上一帧匹配点
        if (match_point_index_ < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("reference_line"), "find match_point_index failed");
            return refer_line_;
        }

        /*
        2. 确定参考线(refer_line_)的下标:
            通过match_point_index_计算refer_line_的最后点和最前点下标;
        */
        const int global_path_size = global_path.poses.size();

        int front_index_tmp = match_point_index_ + reference_line_config_->get_refer_line().front_size_;
        front_index_ = front_index_tmp > (global_path_size - 1) ? (global_path_size - 1) : front_index_tmp;

        int back_index_tmp = match_point_index_ - reference_line_config_->get_refer_line().back_size_;
        back_index_ = back_index_tmp > 0 ? back_index_tmp : 0;

        /*
        3. 填充参考线(refer_line_)的位姿信息:
            将global_path_中refer_line_片段每个点的位姿信息填充进refer_line_;
        */
        ReferlinePoint point_tmp;
        for (int i = back_index_; i <= front_index_; i++)
        {
            point_tmp.pose = global_path.poses[i];
            refer_line_.refer_line.emplace_back(point_tmp);
        }

        /*
        4. 平滑整条参考线
        */
        refer_line_smoother_->smooth_reference_line(refer_line_);

        /*
        5. 填充参考线(refer_line_)的投影点信息:
        */
        Curve::cal_projection_param(refer_line_);

        RCLCPP_INFO(rclcpp::get_logger("reference_line"),
                    "reference line created:\n match_point_index = %d,\n front_index = %d,\n back_index = %d,\n refer_line_size = %ld \n",
                    match_point_index_, front_index_, back_index_, refer_line_.refer_line.size());
        return refer_line_;
    }

    // 转换成rviz显示的Path格式
    Path ReferenceLineCreator::referline_to_rviz()
    {
        refer_line_rviz_.header = refer_line_.header;
        refer_line_rviz_.poses.clear(); // 清除旧数据

        PoseStamped point_tmp;
        for (const ReferlinePoint &point : refer_line_.refer_line)
        {
            point_tmp.header = refer_line_rviz_.header;
            point_tmp.pose = point.pose.pose;
            refer_line_rviz_.poses.emplace_back(point_tmp);
        }

        return refer_line_rviz_;
    }
} // namespace Planning