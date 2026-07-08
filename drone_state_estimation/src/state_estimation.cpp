// Convert pressure to altitude using the barometric formula
// Convert the magnetic field measurements to a heading (yaw) using the magnetometer data and the current roll and pitch angles of the drone
// Use the IMU data to estimate the roll and pitch angles of the drone using a complementary filter or a Kalman filter
// Publish the estimated state of the drone (position, velocity, orientation) to a ROS topic for use by other nodes in the system

#include "drone_state_estimation/state_estimation.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


namespace drone_state_estimation
{
// StateEstimationNode::StateEstimationNode() : Node("state_estimation_node")
StateEstimationNode::StateEstimationNode(const rclcpp::NodeOptions & options) : Node("state_estimation_node", options)
{
  // Get parameters for the topic names
  imu_topic_name = this->declare_parameter<std::string>("imu_topic", "/x500/imu");
  magnetometer_topic_name = this->declare_parameter<std::string>("magnetometer_topic", "/x500/magnetometer");
  barometer_topic_name = this->declare_parameter<std::string>("barometer_topic", "/x500/barometer/air_pressure");

  // Subscribe to the IMU, magnetometer, and barometer topics
  imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_name, 10, std::bind(&StateEstimationNode::imu_callback, this, std::placeholders::_1));
  magnetometer_subscription_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
    magnetometer_topic_name, 10, std::bind(&StateEstimationNode::magnetometer_callback, this, std::placeholders::_1));
  barometer_subscription_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
    barometer_topic_name, 10, std::bind(&StateEstimationNode::barometer_callback, this, std::placeholders::_1));

  // Publisher for the estimated state of the drone
  state_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("estimated_state", 10);
  // Publisher for the estimated altitude of the drone
  altitude_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("x500/altitude", 10);
}

void StateEstimationNode::compute_state_transition_jacobian(const Eigen::Vector3d& accel_measured, 
                                      const Eigen::Vector3d& gyro_measured, 
                                      double dt)
{

  Matrix15d A_;  
  A_.setZero();

  // skew_symmetric
  // R
  auto R = Eigen::Matrix3d::Identity(); // Placeholder for the rotation matrix, replace with actual rotation matrix if available

  A_.block<3, 3>(POS, VEL) = Eigen::Matrix3d::Identity(); // Position to velocity
  A_.block<3, 3>(VEL, ATT) = -R * skewSymmetric(accel_measured); // Velocity to attitude
  A_.block<3, 3>(VEL, B_ACC) = -R; // Velocity to gyroscope bias
  A_.block<3, 3>(ATT, ATT) = -skewSymmetric(gyro_measured); // Velocity to accelerometer bias
  A_.block<3, 3>(ATT, B_GYRO) = -Eigen::Matrix3d::Identity(); // Attitude to gyroscope bias

  F_.setIdentity(); // Initialize F_ as an identity matrix
  F_ += A_ * dt; // Update the state transition Jacobian with the computed A_ matrix

}

void StateEstimationNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
// Process IMU data to estimate roll and pitch angles

  predict_state();
}

void StateEstimationNode::magnetometer_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
{
  // Process magnetometer data to estimate heading (yaw)

}

void StateEstimationNode::barometer_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg)
{
  // Process barometer data to estimate altitude
  // Implementation for altitude estimation goes here
  double pressure = msg->fluid_pressure; // Pressure in Pascals
  double sea_level_pressure = 101325.0; // Standard sea level pressure in Pascals
  double temperature = 288.15; // Standard temperature in Kelvin (15°C)
  double altitude = (temperature / 0.0065) * (1 - pow(pressure / sea_level_pressure, 0.1903)); // Altitude in meters

  auto altitude_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
  altitude_msg.header.stamp = this->get_clock()->now();
  altitude_msg.header.frame_id = "base_link";
  altitude_msg.pose.pose.position.z = altitude; // Set the altitude in the z-axis
  altitude_msg.pose.covariance[14] = 0.1; // Example covariance value for altitude estimation
  altitude_publisher_->publish(altitude_msg);

}

void StateEstimationNode::predict_state()
{
  // Create a state vector and covariance matrix for the drone's state estimation

  F_ = Matrix15d::Zero(); 

  // x_predicted_ = A_ * x_estimated_ + B_ * u_; // Predict the next state based on the previous state and control inputs
  // IMU data can be used to update the control inputs (u_) for the prediction step
  // Update the control inputs based on IMU data
  // Implementation for updating control inputs goes here
  // ...

  // P_cov_predicted_ = A_ * P_cov_estimated_ * A_.transpose() + Q_; // Predict the covariance of the state estimate
  // Predict the state of the drone based on the previous state and control inputs
  // Implementation for state prediction goes here
}

void StateEstimationNode::update_state_with_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // Update the state of the drone based on sensor measurements
  // Implementation for state update goes here

}

} // namespace drone_state_estimation