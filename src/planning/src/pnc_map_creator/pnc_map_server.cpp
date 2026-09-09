#include "pnc_map_server.h"

namespace Planning
{
    PNCMapServer::PNCMapServer() : Node("PNCMapServer_node") // pnc_map服务器
    {
        RCLCPP_INFO(this->get_logger(), "pnc_map_server_node create.");

        // 创建地图发布器对象（消息类型、话题名、队列长度）
        map_pub_ = this->create_publisher<PNCMap>("pnc_map", 10);
        map_rviz_pub_ = this->create_publisher<MarkerArray>("pnc_map_markerarray", 10);

        // 创建地图服务器对象（服务器类型，服务器名，回调函数）
        map_server_ = this->create_service<PNCMapService>(
            "pnc_map_server",
            std::bind(
                &PNCMapServer::response_pnc_map_callback, // 成员函数指针
                this,                                     // 调用成员函数所需的 this 指针
                std::placeholders::_1,                    // 占位符，表示新对象被调用的第一个参数
                std::placeholders::_2                     // 占位符，表示新对象被调用的第二个参数
                ));
    }

    // 响应并发布地图
    void PNCMapServer::response_pnc_map_callback(
        const std::shared_ptr<PNCMapService::Request> request,
        const std::shared_ptr<PNCMapService::Response> response)
    {
        // 接受请求，多态
        switch (request->map_type)
        {
        case static_cast<int>(PNCMapType::STRAIGHT):
            map_creator_ = std::make_shared<PNCMapCreatorStraight>();
            break;
        case static_cast<int>(PNCMapType::STURN):
            map_creator_ = std::make_shared<PNCMapCreatorSTurn>();
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Invalid map_type: %d", request->map_type);
            response->pnc_map = PNCMap(); // 显示返回空路径
            return;
        }

        // 创建并响应地图
        map_creator_->creat_pnc_map(); // 1.填充pnc_map_里的points数据; 2. 填充 pnc_map_markerarray_(给 RViz 用的)
        const PNCMap pnc_map0 = map_creator_->get_pnc_map(); // 仅返回 pnc_map_ 数据, 无上面的填充数据，无法实现画图。
        response->pnc_map = pnc_map0;

        // 发布地图，planning node用
        map_pub_->publish(pnc_map0);
        RCLCPP_INFO(this->get_logger(), "pnc_map published");

        // 发布地图，用于 rviz 显示用
        const MarkerArray pnc_map_markerarray = map_creator_->get_pnc_map_markerarray();
        map_rviz_pub_->publish(pnc_map_markerarray);
        RCLCPP_INFO(this->get_logger(), "pnc_map for rviz published");
    }
} // namespace Planning

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planning::PNCMapServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}