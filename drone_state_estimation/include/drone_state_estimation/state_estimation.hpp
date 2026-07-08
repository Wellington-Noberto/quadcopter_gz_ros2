#ifndef DRONE_STATE_ESTIMATION_HPP
#define DRONE_STATE_ESTIMATION_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include <Eigen/Dense>


namespace drone_state_estimation
{
const int STATE_SIZE = 15;
using Vector15d = Eigen::Matrix<double, STATE_SIZE, 1>;
using Matrix15d = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>;

enum StateIndex
{
  POS = 0,          // Position (x, y, z)
  VEL = 3,          // Velocity (vx, vy, vz)
  ATT = 6,          // Attitude (Euler angles)
  B_ACC = 9,        // Accelerometer bias
  B_GYRO = 12       // Gyroscope bias
};


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
  void compute_state_transition_jacobian(const Eigen::Vector3d& accel_measured, 
                                      const Eigen::Vector3d& gyro_measured, 
                                      double dt);

  inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d skew;
    skew <<   0,   -v.z(),  v.y(),
            v.z(),    0,   -v.x(),
            -v.y(),  v.x(),    0;
    return skew;
  }
  std::string imu_topic_name;
  std::string magnetometer_topic_name;
  std::string barometer_topic_name;
  
  // The unified 15x15 State Covariance Matrix
  Matrix15d P_;    
  // The unified 15x15 State Transition Jacobian
  Matrix15d F_;
  // State vector: [position(3), velocity(3), orientation(3), angular_velocity(3), linear_acceleration(3)]
  Vector15d x_error_states_;
  // Nominal state vector: [position(3), velocity(3), orientation(3), angular_velocity(3), linear_acceleration(3)]
  Vector15d x_nominal_states_;
	
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr magnetometer_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr barometer_subscription_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr altitude_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr state_publisher_;
	

};
}  // namespace drone_state_estimation

#endif  // DRONE_STATE_ESTIMATION_HPP
