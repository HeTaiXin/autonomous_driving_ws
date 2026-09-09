#include "planning_process.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Planning::PlanningProcess>();
  RCLCPP_INFO(node->get_logger(), "planning start! ");

  if(!node->process())
  {
    RCLCPP_ERROR(node->get_logger(), "planning failed!");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
