#ifndef PNC_MAP_CREATOR_BASE_H_
#define PNC_MAP_CREATOR_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/pnc_map.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "config_reader.h"
#include <cmath>

namespace Planning // pnc_map创建器基类
{
    using base_msgs::msg::PNCMap;
    using geometry_msgs::msg::Point;
    using visualization_msgs::msg::Marker;
    using visualization_msgs::msg::MarkerArray;

    enum class PNCMapType // 地图类型
    {
        STRAIGHT,
        STURN,
    };

    class PNCMapCreatorBase // pnc_map创建器基类
    {
    public:
        /* 声明虚析构函数，确保通过基类指针删除派生类对象时，
           能够正确调用派生类的析构函数，防止资源泄漏。 */
        virtual ~PNCMapCreatorBase() = default;
        // 定义一个纯虚函数，作为所有派生类必须实现的接口。
        virtual void creat_pnc_map() = 0; // 生成地图

        inline const PNCMap &get_pnc_map() const { return pnc_map_; }                              // 获取地图
        inline const MarkerArray &get_pnc_map_markerarray() const { return pnc_map_markerarray_; } // 获取rviz用的地图

    protected:                                         // 这里用保护类型，子类中可以继承并访问，但对象中不可访问。
        std::unique_ptr<ConfigReader> pnc_map_config_; // 配置
        int map_type_{0};                              // 类型
        PNCMap pnc_map_;                               // 地图
        MarkerArray pnc_map_markerarray_;              // rviz用的地图

        Point p_mid_, pl_, pr_;  // 左中右三个点
        double theta_current_{}; // 当前角度
        double len_step_{};      // 长度步长
        double theta_step_{};    // 角度步长
    };

} // namespace Planning

#endif // PNC_MAP_CREATOR_BASE_H_