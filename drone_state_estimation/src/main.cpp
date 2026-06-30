#include "drone_state_estimation/state_estimation.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  rclcpp::spin(std::make_shared<drone_state_estimation::StateEstimationNode>(options));
  rclcpp::shutdown();
  return 0;
}