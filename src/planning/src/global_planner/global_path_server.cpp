#include "global_path_server.h"

namespace Planning
{
    GlobalPathServer::GlobalPathServer() // 全局路径服务器
        : Node("GlobalPathServer_node")
    {
        RCLCPP_INFO(this->get_logger(), "global_path_server_node create.");

        // 创建全局路径发布器（消息类型、话题名、队列长度）
        global_path_pub_ = this->create_publisher<Path>("global_path", 10);
        global_path_rviz_pub_ = this->create_publisher<Marker>("global_path_rviz", 10);

        // 创建全局路径服务器（服务器类型、服务器名、回调函数）
        global_path_server_ = this->create_service<GlobalPathService>(
            "global_path_server",
            std::bind(
                &GlobalPathServer::response_global_path_callback, // 成员函数指针
                this,                                             // 调用成员函数所需的 this 指针
                std::placeholders::_1,                            // 占位符，表示新对象被调用的第一个参数
                std::placeholders::_2                             // 占位符，表示新对象被调用的第二个参数
                ));
    }

    void GlobalPathServer::response_global_path_callback(const std::shared_ptr<GlobalPathService::Request> request,
                                                         const std::shared_ptr<GlobalPathService::Response> response)
    {
        // 接受请求，多态
        switch (request->global_planner_type)
        {
        case static_cast<int>(GlobalPlannerType::NORMAL):
            global_planner_creator_ = std::make_shared<GlobalPlannerNormal>();
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Invalid global_planner_type: %d", request->global_planner_type);
            request->global_planner_type = int();   // 返回空数值
            request->pnc_map = PNCMap();            // 返回空路径
            return;
        }

        // 判断请求是否为空
        if (request->pnc_map.midline.points.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "pnc_map empty, global_path cannot be created.");
            return;
        }

        // 搜索并响应全局路径
        global_planner_creator_->search_global_path(request->pnc_map); //给全局路径 global_path_ 赋值
        const Path global_path0 = global_planner_creator_->get_global_path(); // 获取赋值后的全局路径 global_path_
        response->global_path = global_path0;

        // 发布全局路径，给局部规划用
        // 因为只发布1次，并且path没有frame_locked，所以无法固定在rviz中
        global_path_pub_->publish(global_path0);
        RCLCPP_INFO(this->get_logger(), "global_path published");

        // 发布用于rviz显示的全局路径
        const Marker global_path_rviz = path_to_marker(global_path0);
        global_path_rviz_pub_->publish(global_path_rviz);
        RCLCPP_INFO(this->get_logger(), "global_path for rviz published");
    }

    Marker GlobalPathServer::path_to_marker(const Path &path)
    {
        Marker path_rviz_;
        path_rviz_.header = path.header;
        path_rviz_.ns = "global_path";
        path_rviz_.id = 0;
        path_rviz_.action = Marker::ADD;
        path_rviz_.type = Marker::LINE_STRIP;          // 连续的线条
        path_rviz_.scale.x = 0.05;                      // 线段宽度
        path_rviz_.color.a = 1.0;                      // 不透明度
        path_rviz_.color.r = 0.8;                      // 红色
        path_rviz_.color.g = 0.0;                      // 绿色
        path_rviz_.color.b = 0.0;                      // 蓝色
        path_rviz_.lifetime = rclcpp::Duration::max(); // 无限时间
        path_rviz_.frame_locked = true;                // 锁定坐标系

        Point p_tmp;
        for (const auto &pose : path.poses)
        {
            p_tmp.x = pose.pose.position.x;
            p_tmp.y = pose.pose.position.y;
            path_rviz_.points.emplace_back(p_tmp);
        }

        return path_rviz_;
    }

} // namespace Planning

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planning::GlobalPathServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}