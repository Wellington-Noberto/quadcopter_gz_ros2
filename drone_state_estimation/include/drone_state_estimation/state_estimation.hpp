#ifndef DRONE_STATE_ESTIMATION_HPP
#define DRONE_STATE_ESTIMATION_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"


namespace drone_state_estimation
{

class StateEstimationNode : public rclcpp::Node
{
public:
  explicit StateEstimationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  // ~StateEstimationNode();

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void magnetometer_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg);
  void barometer_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg);
  
  void predict_state();
  void update_state_with_imu(const sensor_msgs::msg::Imu::SharedPtr msg);
  

  std::string imu_topic_name;
  std::string magnetometer_topic_name;
  std::string barometer_topic_name;
  
	
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr magnetometer_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr barometer_subscription_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr altitude_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr state_publisher_;
	
};
}  // namespace drone_state_estimation

#endif  // DRONE_STATE_ESTIMATION_HPP
