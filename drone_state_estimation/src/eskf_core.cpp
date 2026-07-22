#include "drone_state_estimation/eskf_core.hpp"


namespace drone_state_estimation
{
MEKF::MEKF()
{
  P_.setIdentity(); // Initialize state covariance matrix
  x_error_states_.setZero(); // Initialize error state vector
}

Eigen::Quaterniond MEKF::compute_delta_q(const Eigen::Vector3d& delta_theta, double dt) const
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
                    double dt) const
{
  NominalState x_next = x;

  Eigen::Vector3d pos      = x.pos;
  Eigen::Vector3d vel      = x.vel;
  Eigen::Quaterniond attitude_q = x.att_quat;
  Eigen::Vector3d accel_corrected = accel_measured - x.b_acc;
  Eigen::Vector3d gyro_corrected  = gyro_measured  - x.b_gyro;

  Eigen::Matrix3d R = attitude_q.toRotationMatrix();

  Eigen::Vector3d gravity(0, 0, -9.81);  // Gravity vector in the world frame (>> assuming z is up <<)

  x_next.pos = pos + vel * dt;
  x_next.vel = vel + (R * accel_corrected + gravity) * dt;
  x_next.att_quat = (attitude_q *  compute_delta_q(gyro_corrected, dt)).normalized();  // << Correct here
  x_next.b_acc = x.b_acc;
  x_next.b_gyro = x.b_gyro;

  return x_next;
}

ErrorStateMatrix
MEKF::compute_state_jacobian(const NominalState& x,
                            const Eigen::Vector3d& accel_measured, 
                            const Eigen::Vector3d& gyro_measured, 
                            double dt) const
{

  ErrorStateMatrix A;  
  A.setZero();
  // Compute true states
  Eigen::Vector3d accel_corrected = accel_measured - x.b_acc;
  Eigen::Vector3d gyro_corrected  = gyro_measured  - x.b_gyro;

  // Compute Rotation matrix from Euler angles
  Eigen::Quaterniond attitude_q = x.att_quat;
  Eigen::Matrix3d R = attitude_q.toRotationMatrix();

  A.block<3, 3>(POS, VEL) = Eigen::Matrix3d::Identity(); // Position to velocity
  A.block<3, 3>(VEL, ATT) = -R * skewSymmetric(accel_corrected); // Velocity to attitude
  A.block<3, 3>(VEL, B_ACC) = -R; // Velocity to gyroscope bias
  A.block<3, 3>(ATT, ATT) = -skewSymmetric(gyro_corrected); // Velocity to accelerometer bias
  A.block<3, 3>(ATT, B_GYRO) = -Eigen::Matrix3d::Identity(); // Attitude to gyroscope bias

  ErrorStateMatrix F = ErrorStateMatrix::Identity();
  F += A * dt + 1/2 * A * A * dt * dt; // Update the state transition Jacobian with the computed A matrix
  // ToDo: Second order approximation
  return F;
}

ProcessNoiseCovariance MEKF::initialize_process_noise_covariance(const NoiseStdDev& process_noise_stddev) const
{
  ProcessNoiseCovariance Qc;
  Qc.setZero(); 

  Qc.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * process_noise_stddev.accel_noise_stddev.squaredNorm();
  Qc.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * process_noise_stddev.gyro_noise_stddev.squaredNorm();
  Qc.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * process_noise_stddev.gyro_bias_noise_stddev.squaredNorm();
  Qc.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * process_noise_stddev.accel_bias_noise_stddev.squaredNorm();

  return Qc;
}

NoiseJacobian MEKF::compute_process_noise_jacobian(const NominalState& x) const
{
  NoiseJacobian L{NoiseJacobian::Zero()};
  Eigen::Matrix3d R = x.att_quat.toRotationMatrix();

  L.block<3, 3>(3, 3)  = -R;
  L.block<3, 3>(6, 0)  = -Eigen::Matrix3d::Identity();
  L.block<3, 3>(9, 6)  =  Eigen::Matrix3d::Identity();
  L.block<3, 3>(12, 9) =  Eigen::Matrix3d::Identity();

  return L;
}

ErrorStateMatrix MEKF::compute_process_noise_covariance(const NominalState& x,
                                                        const NoiseStdDev& noise_params,
                                                        double dt) const
{
  ProcessNoiseCovariance Qc = initialize_process_noise_covariance(noise_params);
  NoiseJacobian L = compute_process_noise_jacobian(x);
  
  return L * Qc * L.transpose() * dt;   // First order approximation of the process noise covariance
}

ErrorStateMatrix MEKF::propagate_covariance(const ErrorStateMatrix& P, 
                                            const ErrorStateMatrix& F, 
                                            const ErrorStateMatrix& Q) const
{
  return F * P * F.transpose() + Q;  // Propagate the state covariance matrix
}

void MEKF::predict(const Eigen::Vector3d& accel_measured,
                  const Eigen::Vector3d& gyro_measured,
                  const NoiseStdDev& noise_params,
                  double dt)
{
  // Implement the prediction step of the MEKF here
  // std_msgs/Header header
  // string child_frame_id
  // geometry_msgs/PoseWithCovariance pose
  // geometry_msgs/TwistWithCovariance twist


  const ErrorStateMatrix F = compute_state_jacobian(x_, accel_measured, gyro_measured, dt);
  const ErrorStateMatrix Q = compute_process_noise_covariance(x_, noise_params, dt);

  x_ = predict_states(x_, accel_measured, gyro_measured, dt);
  P_ = propagate_covariance(P_, F, Q);
}


} // namespace drone_state_estimation



