#ifndef VEHICLE_INFO_BASE_H_
#define VEHICLE_INFO_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/referline.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "config_reader.h"
#include "curve.h"

namespace Planning
{
    using base_msgs::msg::Referline;
    using geometry_msgs::msg::PoseStamped;
    using geometry_msgs::msg::TransformStamped;
    using nav_msgs::msg::Path;

    class VehicleInfoBase // 车辆基类
    {
    public:
        // 更新参数
        inline void updata_location(const PoseStamped &loc) { loc_point_ = loc; }

        // 虚析构
        virtual ~VehicleInfoBase() = default;
        // 定位点转frenet
        virtual void vehicle_cartesian_to_frenet(const Referline &refer_line) = 0; // 定位点在参考线上的投影点参数

        // 基本属性
        inline const std::string &get_child_frame() const { return child_frame_; }
        inline const double &get_length() const { return length_; }
        inline const double &get_width() const { return width_; }
        inline const int &get_id() const { return id_; }

        // 笛卡尔(Cartesian)参数
        inline const PoseStamped &get_loc_point() const { return loc_point_; }
        inline const double &get_theta() const { return theta_; }
        inline const double &get_kappa() const { return kappa_; }
        inline const double &get_dkappa() const { return dkappa_; }
        inline const double &get_speed() const { return speed_; }
        inline const double &get_acceleration() const { return acceleration_; }
        inline const double &get_dacceleration() const { return dacceleration_; }

        // 向参考线投影的frenet参数
        inline const double &get_s() const { return s_; }
        inline const double &get_l() const { return l_; }
        inline const double &get_ds_dt() const { return ds_dt_; }
        inline const double &get_dl_dt() const { return dl_dt_; }
        inline const double &get_dl_ds() const { return dl_ds_; }
        inline const double &get_ddl_dt() const { return ddl_dt_; }
        inline const double &get_ddl_ds() const { return ddl_ds_; }
        inline const double &get_dds_dt() const { return dds_dt_; }

        // 向路径投影的frenet参数

        // 时间参数

    protected:
        // 基本属性
        std::unique_ptr<ConfigReader> vehicle_config_; // 配置
        std::string child_frame_;                      // 坐标名
        double length_{0.0};                           // 长
        double width_{0.0};                            // 宽
        int id_{0};                                    // 序号

        // 笛卡尔(Cartesian)参数
        PoseStamped loc_point_;     // 定位点
        double theta_{0.0};         // 航向角
        double kappa_{0.0};         // 曲率
        double dkappa_{0.0};        // 曲率变化率
        double speed_{0.0};         // 速度
        double acceleration_{0.0};  // 加速度
        double dacceleration_{0.0}; // 加加速度

        // 向参考线投影的frenet参数
        double s_{0.0};
        double l_{0.0};
        double ds_dt_{0.0};
        double dl_ds_{0.0};
        double dl_dt_{0.0};
        double dds_dt_{0.0};
        double ddl_ds_{0.0};
        double ddl_dt_{0.0};

        // 向路径投影的frenet参数

        // 时间参数
    };
} // namespace Planner

#endif // VEHICLE_INFO_BASE_H_