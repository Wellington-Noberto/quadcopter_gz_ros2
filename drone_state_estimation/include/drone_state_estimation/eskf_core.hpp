#ifndef ESKF_CORE_HPP
#define ESKF_CORE_HPP

#include <Eigen/Dense>

namespace drone_state_estimation
{
  
const int STATE_SIZE = 15;
using ErrorStateVector = Eigen::Matrix<double, STATE_SIZE, 1>;
using ErrorStateMatrix = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>;

struct NominalState {
  Eigen::Vector3d pos{Eigen::Vector3d::Zero()};  // Position (x, y, z)
  Eigen::Vector3d vel{Eigen::Vector3d::Zero()};  // Velocity (vx, vy, vz)
  Eigen::Quaterniond att_quat{Eigen::Quaterniond::Identity()};  // Attitude (quaternion)
  Eigen::Vector3d b_acc{Eigen::Vector3d::Zero()};  // Accelerometer bias
  Eigen::Vector3d b_gyro{Eigen::Vector3d::Zero()};  // Gyroscope bias
};

enum StateIndex
{
  POS = 0,          // Position (x, y, z)
  VEL = 3,          // Velocity (vx, vy, vz)
  ATT = 6,          // Attitude (Euler angles)
  B_ACC = 9,        // Accelerometer bias
  B_GYRO = 12       // Gyroscope bias
};


class MEKF
{
public:
  MEKF();
  void predict(const Eigen::VectorXd& u, double dt);
  void update(const Eigen::VectorXd& z);

  NominalState predict_states(const NominalState& x,
                              const Eigen::Vector3d& accel_measured,
                              const Eigen::Vector3d& gyro_measured,
                              double dt);

  Eigen::Quaterniond compute_delta_q(const Eigen::Vector3d& delta_theta, double dt);

  ErrorStateMatrix compute_state_jacobian(const NominalState& x,
                                         const Eigen::Vector3d& accel_measured, 
                                         const Eigen::Vector3d& gyro_measured, 
                                         double dt);


  inline Eigen::Matrix3d rotationMatrixFromEulerAngles(const Eigen::Vector3d& euler_angles) {
    double roll = euler_angles(0);
    double pitch = euler_angles(1);
    double yaw = euler_angles(2);

    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    return R;
  }

  inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d skew;
    skew <<   0,   -v.z(),  v.y(),
            v.z(),    0,   -v.x(),
            -v.y(),  v.x(),    0;
    return skew;
  }


private:
  // State Covariance Matrix
  ErrorStateMatrix P_;    
  // State Transition Jacobian
  ErrorStateMatrix F_;
  // State vector: [position(3), velocity(3), orientation(3), angular_velocity(3), linear_acceleration(3)]
  ErrorStateVector x_error_states_;
  // Nominal state vector: [position(3), velocity(3), orientation(3), angular_velocity(3), linear_acceleration(3)]
  ErrorStateVector x_nominal_states_;
};

}  // namespace drone_state_estimation

#endif  // ESKF_CORE_HPP