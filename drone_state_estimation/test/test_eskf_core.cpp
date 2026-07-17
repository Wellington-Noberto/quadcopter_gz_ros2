#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "drone_state_estimation/eskf_core.hpp"

TEST(MEKF, TestRotationMatrixFromEulerAngles)
{
    drone_state_estimation::MEKF kf;

    Eigen::Vector3d euler_angles(0.1, 0.2, 0.3); // roll, pitch, yaw
    Eigen::Matrix3d R = kf.rotationMatrixFromEulerAngles(euler_angles);

    // Expected rotation matrix computed manually or from a reliable source
    Eigen::Matrix3d expected_R;
    expected_R << 0.93629336, -0.27509585,  0.21835066,
                  0.28962948,  0.95642509, -0.03695701,
                 -0.19866933,  0.0978434 ,  0.97517033;

    std::cout << "Computed rotation matrix:\n" << R << std::endl;
    std::cout << "Expected rotation matrix:\n" << expected_R << std::endl;
    std::cout << "Difference:\n" << R - expected_R << std::endl;

    EXPECT_TRUE(R.isApprox(expected_R, 1e-6));
}

TEST(MEKF, TestSkewSymmetric)
{
    drone_state_estimation::MEKF kf;

    Eigen::Vector3d v(1.0, 2.0, 3.0);
    Eigen::Matrix3d skew = kf.skewSymmetric(v);

    Eigen::Matrix3d expected_skew;
    expected_skew << 0, -3, 2,
                     3, 0, -1,
                     -2, 1, 0;

    std::cout << "Computed skew-symmetric matrix:\n" << skew << std::endl;
    std::cout << "Expected skew-symmetric matrix:\n" << expected_skew << std::endl;
    std::cout << "Difference:\n" << skew - expected_skew << std::endl;
    EXPECT_TRUE(skew.isApprox(expected_skew));
}

TEST(MEKF, IdentityAtZeroDt)
{
  drone_state_estimation::MEKF kf;
  drone_state_estimation::NominalState x = drone_state_estimation::NominalState();
  drone_state_estimation::ErrorStateMatrix F = kf.compute_state_jacobian(x, Eigen::Vector3d(1,2,3), Eigen::Vector3d(0.1,0.2,0.3), 0.0);
  EXPECT_TRUE(F.isApprox(drone_state_estimation::ErrorStateMatrix::Identity(), 1e-9));
}

TEST(MEKF, NoNaNs) 
{
  drone_state_estimation::MEKF kf;
  drone_state_estimation::NominalState x = drone_state_estimation::NominalState();
  drone_state_estimation::ErrorStateMatrix F = kf.compute_state_jacobian(x, Eigen::Vector3d(0,0,9.81), Eigen::Vector3d::Zero(), 0.01);

  std::cout << "Nominal states:\n" 
            << "Position: " << x.pos.transpose() 
            << "\nVelocity: " << x.vel.transpose() 
            << "\nAttitude (quaternion): " << x.att_quat 
            << "\nAccelerometer bias: " << x.b_acc.transpose() 
            << "\nGyroscope bias: " << x.b_gyro.transpose() << std::endl;

  std::cout << "Computed state transition Jacobian:\n" << F << std::endl;
  EXPECT_FALSE(F.hasNaN());
}

TEST(MEKF, TestNonLinearPrediction) 
{
  drone_state_estimation::MEKF kf;
  drone_state_estimation::NominalState x;
  x.pos << 1.0, 1.0, 1.0;

  drone_state_estimation::NominalState x_minus = x;
  x_minus.pos << 0.9, 0.9, 0.9;

  drone_state_estimation::NominalState x_pred = kf.predict_states(x, Eigen::Vector3d(0,0,9.81), Eigen::Vector3d::Zero(), 0.01);
  drone_state_estimation::NominalState x_minus_pred = kf.predict_states(x_minus, Eigen::Vector3d(0,0,9.81), Eigen::Vector3d::Zero(), 0.01);

  std::cout << "Predicted state from x:\n" << "Position: " << x_pred.pos.transpose() << std::endl;
  std::cout << "Predicted state from x_minus:\n" << "Position: " << x_minus_pred.pos.transpose() << std::endl;

  EXPECT_FALSE(x_pred.pos.hasNaN());
}

TEST(MEKF, TestStateTransitionJacobian)
{
  // Increment the nominal state by a small error state
  auto states_increment = [&](const drone_state_estimation::NominalState& x, const drone_state_estimation::ErrorStateVector& dx) -> drone_state_estimation::NominalState {
    drone_state_estimation::NominalState x_perturbed = x;
    x_perturbed.pos     = x.pos     + dx.segment<3>(drone_state_estimation::POS);
    x_perturbed.vel     = x.vel     + dx.segment<3>(drone_state_estimation::VEL);
    x_perturbed.b_acc   = x.b_acc   + dx.segment<3>(drone_state_estimation::B_ACC);
    x_perturbed.b_gyro  = x.b_gyro  + dx.segment<3>(drone_state_estimation::B_GYRO);

    Eigen::Vector3d dtheta = dx.segment<3>(drone_state_estimation::ATT);
    Eigen::Quaterniond dq = Eigen::Quaterniond(Eigen::AngleAxisd(dtheta.norm(), dtheta.normalized()));
    x_perturbed.att_quat = (x.att_quat * dq).normalized();

    return x_perturbed;
  };

  // Compute the difference between two nominal states
  auto states_difference = [&](drone_state_estimation::NominalState x1, drone_state_estimation::NominalState x2) -> drone_state_estimation::ErrorStateVector {
    drone_state_estimation::ErrorStateVector diff;
    diff.segment<3>(drone_state_estimation::POS) = x1.pos - x2.pos;
    diff.segment<3>(drone_state_estimation::VEL) = x1.vel - x2.vel;

    Eigen::Quaterniond q2_inv = x2.att_quat.conjugate();
    Eigen::Quaterniond q_diff = q2_inv * x1.att_quat;
    if (q_diff.w() < 0.0) {
      q_diff.coeffs() = -q_diff.coeffs();  // flip to the equivalent quaternion with positive scalar part
    }
    Eigen::AngleAxisd aa(q_diff); 
    diff.segment<3>(drone_state_estimation::ATT) = aa.axis() * aa.angle();

    diff.segment<3>(drone_state_estimation::B_ACC) = x1.b_acc - x2.b_acc;
    diff.segment<3>(drone_state_estimation::B_GYRO) = x1.b_gyro - x2.b_gyro;

    return diff;
  };

  drone_state_estimation::MEKF kf;
  drone_state_estimation::NominalState x = drone_state_estimation::NominalState();
  x.pos << 1.0, 0.0, 0.0;
  x.vel << 0.5, 0.1, 0.0;
  x.att_quat = Eigen::Quaterniond(1, 0.05, 0.02, 0.1).normalized();   // nonzero, avoids masking sign errors
  x.b_acc << 0.01, -0.02, 0.005;
  x.b_gyro << 0.001, 0.002, -0.001;

  Eigen::Vector3d accel_measured(0.1, 0.2, 9.81);
  Eigen::Vector3d gyro_measured(0.01, 0.02, 0.03);
  double dt = 0.01;

  drone_state_estimation::ErrorStateMatrix F_computed = kf.compute_state_jacobian(x, accel_measured, gyro_measured, dt);
  drone_state_estimation::ErrorStateMatrix F_numeric = drone_state_estimation::ErrorStateMatrix::Zero();
  const double eps = 1e-6;

  for (int i = 0; i < drone_state_estimation::STATE_SIZE; ++i) {
    drone_state_estimation::ErrorStateVector dx_plus  = drone_state_estimation::ErrorStateVector::Zero();
    drone_state_estimation::ErrorStateVector dx_minus = drone_state_estimation::ErrorStateVector::Zero();
    dx_plus(i)  =  eps;
    dx_minus(i) = -eps;

    drone_state_estimation::NominalState x_plus  = states_increment(x, dx_plus);
    drone_state_estimation::NominalState x_minus = states_increment(x, dx_minus);
      
    drone_state_estimation::NominalState x_pred_plus  = kf.predict_states(x_plus,  accel_measured, gyro_measured, dt);
    drone_state_estimation::NominalState x_pred_minus = kf.predict_states(x_minus, accel_measured, gyro_measured, dt);

    F_numeric.col(i) = states_difference(x_pred_plus, x_pred_minus) / (2.0 * eps);
  }

  std::cout << "Computed state transition Jacobian:\n" << F_computed << std::endl;
  std::cout << "Numeric state transition Jacobian:\n" << F_numeric << std::endl;
  std::cout << "Difference:\n" << F_computed - F_numeric << std::endl;

  EXPECT_TRUE(F_computed.isApprox(F_numeric, 1e-5));
}

TEST(MEKF, ProcessNoiseCovarianceIsSymmetricPSD)
{
  drone_state_estimation::MEKF kf;
  drone_state_estimation::NominalState x;  // identity/zero default
  drone_state_estimation::NoiseStdDev noise;
  noise.accel_noise_stddev   = Eigen::Vector3d::Constant(0.05);
  noise.gyro_noise_stddev    = Eigen::Vector3d::Constant(0.01);
  noise.accel_bias_noise_stddev = Eigen::Vector3d::Constant(0.001);
  noise.gyro_bias_noise_stddev  = Eigen::Vector3d::Constant(0.0001);

  drone_state_estimation::ErrorStateMatrix Q = kf.compute_process_noise_covariance(x, noise, 0.01);

  EXPECT_TRUE(Q.isApprox(Q.transpose(), 1e-12));

  Eigen::SelfAdjointEigenSolver<drone_state_estimation::ErrorStateMatrix> solver(Q);
  EXPECT_TRUE((solver.eigenvalues().array() >= -1e-9).all());
}

TEST(MEKF, ProcessNoiseCovarianceScalesLinearlyWithDt)
{
  drone_state_estimation::MEKF kf;
  drone_state_estimation::NominalState x;
  drone_state_estimation::NoiseStdDev noise;  
  noise.accel_noise_stddev   = Eigen::Vector3d::Constant(0.05);
  noise.gyro_noise_stddev    = Eigen::Vector3d::Constant(0.01);
  noise.accel_bias_noise_stddev = Eigen::Vector3d::Constant(0.001);
  noise.gyro_bias_noise_stddev  = Eigen::Vector3d::Constant(0.0001);

  drone_state_estimation::ErrorStateMatrix Q1 = kf.compute_process_noise_covariance(x, noise, 0.01);
  drone_state_estimation::ErrorStateMatrix Q2 = kf.compute_process_noise_covariance(x, noise, 0.02);

  EXPECT_TRUE(Q2.isApprox(2.0 * Q1, 1e-9)); 
}

TEST(MEKF, ProcessNoiseCovarianceHasNoDirectPositionBlock)
{
  drone_state_estimation::MEKF kf;
  drone_state_estimation::NominalState x;
  drone_state_estimation::NoiseStdDev noise;
  noise.accel_noise_stddev = Eigen::Vector3d::Constant(0.05);
  drone_state_estimation::ErrorStateMatrix Q = kf.compute_process_noise_covariance(x, noise, 0.01);
  bool is_zero = Q.block<3, 3>(0, 0).isZero(1e-12);

  EXPECT_TRUE(is_zero);
}