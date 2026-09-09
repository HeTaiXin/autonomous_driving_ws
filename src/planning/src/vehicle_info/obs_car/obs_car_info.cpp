#include "obs_car_info.h"

namespace Planning
{
    ObsCarInfo::ObsCarInfo(const int &id)  // 障碍物
    {
        RCLCPP_INFO(rclcpp::get_logger("vehicle"), "obs_car_info created.");

        // 读取配置
        vehicle_config_ = std::make_unique<ConfigReader>();
        vehicle_config_->read_vehicles_config(); // 包含read_pnc_map_config()

        // 更新基本属性
        child_frame_ = vehicle_config_->obs_pari()[id].frame_;
        length_ = vehicle_config_->obs_pari()[id].length_;
        width_ = vehicle_config_->obs_pari()[id].width_;
        theta_ = vehicle_config_->obs_pari()[id].pose_theta_;
        speed_ = vehicle_config_->obs_pari()[id].speed_ori_;
        id_ = id;

        // 初始化定位点
        tf2::Quaternion qtn;
        qtn.setRPY(0.0, 0.0, theta_);
        loc_point_.header.frame_id = vehicle_config_->get_pnc_map().frame_;
        loc_point_.header.stamp = rclcpp::Clock().now();
        loc_point_.pose.position.x = vehicle_config_->obs_pari()[id].pose_x_;
        loc_point_.pose.position.y = vehicle_config_->obs_pari()[id].pose_y_;
        loc_point_.pose.orientation.x = qtn.getX();
        loc_point_.pose.orientation.y = qtn.getY();
        loc_point_.pose.orientation.z = qtn.getZ();
        loc_point_.pose.orientation.w = qtn.getW();
    }

    void ObsCarInfo::vehicle_cartesian_to_frenet(const Referline &refer_line)
    {
        // TODO: 后续实现障碍物在参考线上的投影及Frenet参数计算
        // 目前仅打印日志或留空
        RCLCPP_DEBUG(rclcpp::get_logger("vehicle"), "obs %d cartesian_to_frenet called (not implemented yet)", id_);
    }
} // Planning