#ifndef GLOBAL_PATH_SERVER_H_
#define GLOBAL_PATH_SERVER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/srv/global_path_service.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "global_planner_normal.h"

namespace Planning
{
    using base_msgs::srv::GlobalPathService;
    using geometry_msgs::msg::Point;
    using visualization_msgs::msg::Marker;

    class GlobalPathServer : public rclcpp::Node
    {
    public:
        GlobalPathServer(); // 全局路径服务器

    private:
        // 全局路径回调
        void response_global_path_callback(const std::shared_ptr<GlobalPathService::Request> request,
                                           const std::shared_ptr<GlobalPathService::Response> response);
        Marker path_to_marker(const Path &path); // Path转Marker，这样才能发给rviz2

        std::shared_ptr<GlobalPlannerBase> global_planner_creator_;         // 全局路径创建器
        rclcpp::Publisher<Path>::SharedPtr global_path_pub_;                // 全局路径发布器
        rclcpp::Publisher<Marker>::SharedPtr global_path_rviz_pub_;         // 给rviz的全局路径发布器
        rclcpp::Service<GlobalPathService>::SharedPtr global_path_server_;  // 全局路径服务器
    };
} // namespace Planning

#endif // GLOBAL_PATH_SERVER_H_