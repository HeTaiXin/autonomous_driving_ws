#ifndef PLANNING_PROCESS_H_
#define PLANNING_PROCESS_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "main_car_info.h"
#include "obs_car_info.h"
#include "reference_line_creator.h"
#include "decision_center.h"
#include "local_path_planner.h"
#include "local_speeds_planner.h"
#include "local_trajectory_combiner.h"

#include "base_msgs/msg/pnc_map.hpp"
#include "base_msgs/srv/pnc_map_service.hpp"
#include "base_msgs/srv/global_path_service.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include <vector>
#include <cmath>
#include <algorithm>

namespace Planning
{
    using namespace std::chrono_literals;
    using base_msgs::msg::PNCMap;
    using base_msgs::srv::GlobalPathService;
    using base_msgs::srv::PNCMapService;
    using geometry_msgs::msg::PoseStamped;
    using nav_msgs::msg::Path;
    using tf2_ros::Buffer;
    using tf2_ros::StaticTransformBroadcaster;
    using tf2_ros::TransformListener;

    class PlanningProcess : public rclcpp::Node // 规划总流程
    {
    public:
        PlanningProcess();
        bool process(); // 总流程

    private:
        bool planning_init();                                                // 流程初始化
        void vehicle_spawn(const std::shared_ptr<VehicleInfoBase> &vehicle); // 生成车辆
        void get_location(const std::shared_ptr<VehicleInfoBase> &vehicle);  // 监听定位点

        template <typename T>
        bool connect_server(const T &client); // 链接服务器
        bool map_request();                   // 发送地图请求
        bool global_path_request();           // 发送全局路径请求
        void planning_callback();             // 总流程的回调函数

    public:
        inline const PNCMap &get_pnc_map() const { return pnc_map_; }       // 获取地图
        inline const Path &get_global_path() const { return global_path_; } // 获取全局路径

    private:
        std::unique_ptr<ConfigReader> process_config_;              // 配置
        std::shared_ptr<VehicleInfoBase> car_;                      // 主车
        std::vector<std::shared_ptr<VehicleInfoBase>> obses_spawn_; // 所有障碍物,模拟感知信号
        std::vector<std::shared_ptr<VehicleInfoBase>> obses_;       // 要考虑的障碍物
        double obs_dis_{0.0};                                       // 考虑障碍物的距离

        std::shared_ptr<StaticTransformBroadcaster> tf_broadcaster_; // 坐标广播器
        std::unique_ptr<Buffer> buffer_;                             // 缓存对象
        std::shared_ptr<TransformListener> tf_listener_;             // 位置监听器

        PNCMap pnc_map_;                                                  // 地图
        Path global_path_;                                                // 全局路径
        rclcpp::Client<PNCMapService>::SharedPtr map_client_;             // 地图请求客户端
        rclcpp::Client<GlobalPathService>::SharedPtr global_path_client_; // 全局路径请求客户端

        std::shared_ptr<ReferenceLineCreator> refer_line_creator_; // 参考线创建器
        rclcpp::Publisher<Path>::SharedPtr refer_line_pub_;        // 参考线发布器

        rclcpp::TimerBase::SharedPtr time_; // 定时器
    };
} // namespace Planning

#endif // PLANNING_PROCESS_H_
