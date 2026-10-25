#include <rclcpp/rclcpp.hpp>
#include <plan_manage/ego_replan_fsm.h>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ego_planner_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  ego_planner::EGOReplanFSM planner;
  planner.init(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
