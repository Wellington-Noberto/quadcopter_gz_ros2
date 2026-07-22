#ifndef ESKF_CORE_HPP
#define ESKF_CORE_HPP

#include <Eigen/Dense>

namespace drone_state_estimation
{
  
const int STATE_SIZE = 15;
const int NOISE_SIZE = 12;
using ErrorStateVector       = Eigen::Matrix<double, STATE_SIZE, 1>;
using ErrorStateMatrix       = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>;
using ProcessNoiseCovariance = Eigen::Matrix<double, NOISE_SIZE, NOISE_SIZE>;
using NoiseJacobian          = Eigen::Matrix<double, STATE_SIZE, NOISE_SIZE>;

struct NominalState {
  Eigen::Vector3d pos{Eigen::Vector3d::Zero()};  // Position (x, y, z)
  Eigen::Vector3d vel{Eigen::Vector3d::Zero()};  // Velocity (vx, vy, vz)
  Eigen::Quaterniond att_quat{Eigen::Quaterniond::Identity()};  // Attitude (quaternion)
  Eigen::Vector3d b_acc{Eigen::Vector3d::Zero()};  // Accelerometer bias
  Eigen::Vector3d b_gyro{Eigen::Vector3d::Zero()};  // Gyroscope bias
};

struct NoiseStdDev {
  Eigen::Vector3d accel_noise_stddev{Eigen::Vector3d::Zero()};  // Accelerometer noise standard deviation
  Eigen::Vector3d gyro_noise_stddev{Eigen::Vector3d::Zero()};  // Gyroscope noise standard deviation
  Eigen::Vector3d accel_bias_noise_stddev{Eigen::Vector3d::Zero()};  // Accelerometer bias noise standard deviation
  Eigen::Vector3d gyro_bias_noise_stddev{Eigen::Vector3d::Zero()};  // Gyroscope bias noise standard deviation
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
  void predict(const Eigen::Vector3d& accel_measured,
               const Eigen::Vector3d& gyro_measured,
               const NoiseStdDev& noise_params, double dt);
  void update(const Eigen::VectorXd& z);

  NominalState predict_states(const NominalState& x,
                              const Eigen::Vector3d& accel_measured,
                              const Eigen::Vector3d& gyro_measured,
                              double dt) const;

  Eigen::Quaterniond compute_delta_q(const Eigen::Vector3d& delta_theta, double dt) const;

  ErrorStateMatrix compute_state_jacobian(const NominalState& x,
                                         const Eigen::Vector3d& accel_measured, 
                                         const Eigen::Vector3d& gyro_measured, 
                                         double dt) const;

  
  
  ProcessNoiseCovariance initialize_process_noise_covariance(const NoiseStdDev& process_noise_stddev) const;

  NoiseJacobian compute_process_noise_jacobian(const NominalState& x) const;
  
  ErrorStateMatrix compute_process_noise_covariance(const NominalState& x,
                                                    const NoiseStdDev& noise_params,
                                                    double dt) const;

  ErrorStateMatrix propagate_covariance(const ErrorStateMatrix& P, 
                                      const ErrorStateMatrix& F, 
                                      const ErrorStateMatrix& ) const;



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

  inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) const{
    Eigen::Matrix3d skew;
    skew <<   0,   -v.z(),  v.y(),
            v.z(),    0,   -v.x(),
            -v.y(),  v.x(),    0;
    return skew;
  }


private:
  // State Covariance Matrix
  ErrorStateMatrix P_;    

  // State vector: [position(3), velocity(3), orientation(3), angular_velocity(3), linear_acceleration(3)]
  ErrorStateVector x_error_states_;
  // Nominal state vector: [position(3), velocity(3), orientation(3), angular_velocity(3), linear_acceleration(3)]
  NominalState x_;
};

}  // namespace drone_state_estimation

#endif  // ESKF_CORE_HPP