#include "drone_state_estimation/eskf_core.hpp"


namespace drone_state_estimation
{
MEKF::MEKF()
{
  P_.setIdentity(); // Initialize state covariance matrix
  F_.setIdentity(); // Initialize state transition Jacobian
  x_error_states_.setZero(); // Initialize error state vector
  x_nominal_states_.setZero(); // Initialize nominal state vector
}

Eigen::Quaterniond MEKF::compute_delta_q(const Eigen::Vector3d& delta_theta, double dt)
{
  Eigen::Vector3d phi = delta_theta * dt;
  double angle = phi.norm();
  if (angle < 1e-8) // avoid division by zero for near-zero rotation; first-order approx is used
  {  
    Eigen::Quaterniond dq(1.0, 0.5 * delta_theta.x(), 0.5 * delta_theta.y(), 0.5 * delta_theta.z());
    dq.normalize();
    return dq;
  } 
  Eigen::Vector3d axis = phi.normalized();
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
}

NominalState 
MEKF::predict_states(const NominalState& x,
                    const Eigen::Vector3d& accel_measured,
                    const Eigen::Vector3d& gyro_measured,
                    double dt)
{
  NominalState x_next = x;

  Eigen::Vector3d pos      = x.pos;
  Eigen::Vector3d vel      = x.vel;
  Eigen::Quaterniond attitude_q = x.att_quat;
  Eigen::Vector3d b_acc    = x.b_acc;
  Eigen::Vector3d b_gyro   = x.b_gyro;

  Eigen::Matrix3d R = attitude_q.toRotationMatrix();

  Eigen::Vector3d accel_corrected = accel_measured - b_acc;
  Eigen::Vector3d gyro_corrected  = gyro_measured  - b_gyro;

  Eigen::Vector3d gravity(0, 0, -9.81);  // Gravity vector in the world frame (>> assuming z is up <<)

  x_next.pos = pos + vel * dt;
  x_next.vel = vel + (R * accel_corrected + gravity) * dt;
  x_next.att_quat = (attitude_q *  compute_delta_q(gyro_corrected, dt)).normalized();  // << Correct here
  x_next.b_acc = b_acc;
  x_next.b_gyro = b_gyro;

  return x_next;
}

ErrorStateMatrix
MEKF::compute_state_jacobian(const NominalState& x,
                            const Eigen::Vector3d& accel_measured, 
                            const Eigen::Vector3d& gyro_measured, 
                            double dt)
{

  ErrorStateMatrix A;  
  A.setZero();
  // Compute true states
  Eigen::Vector3d b_acc    = x.b_acc;
  Eigen::Vector3d b_gyro   = x.b_gyro;
  Eigen::Vector3d accel_corrected = accel_measured - b_acc;
  Eigen::Vector3d gyro_corrected  = gyro_measured  - b_gyro;

  // Compute Rotation matrix from Euler angles
  Eigen::Quaterniond attitude_q = x.att_quat;
  Eigen::Matrix3d R = attitude_q.toRotationMatrix();

  A.block<3, 3>(POS, VEL) = Eigen::Matrix3d::Identity(); // Position to velocity
  A.block<3, 3>(VEL, ATT) = -R * skewSymmetric(accel_corrected); // Velocity to attitude
  A.block<3, 3>(VEL, B_ACC) = -R; // Velocity to gyroscope bias
  A.block<3, 3>(ATT, ATT) = -skewSymmetric(gyro_corrected); // Velocity to accelerometer bias
  A.block<3, 3>(ATT, B_GYRO) = -Eigen::Matrix3d::Identity(); // Attitude to gyroscope bias

  ErrorStateMatrix F = ErrorStateMatrix::Identity();
  F += A * dt; // Update the state transition Jacobian with the computed A matrix

  return F;
}

}