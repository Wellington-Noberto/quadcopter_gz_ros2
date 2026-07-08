#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "drone_state_estimation/state_estimation.hpp"


// TODO: CREATE A SEPARATED CLASS FOR EKF CORE
TEST(EKFCore, TestSkewSymmetric)
{

}

TEST(EKFCore, TestStateTransitionJacobian)
{
    drone_state_estimation::StateEstimationNode node;

    Eigen::Vector3d accel_measured(0.1, 0.2, 9.81);
    Eigen::Vector3d gyro_measured(0.01, 0.02, 0.03);
    double dt = 0.01;

    node.compute_state_transition_jacobian(accel_measured, gyro_measured, dt);

    // Check if the state transition Jacobian is computed correctly
    Eigen::Matrix<double, 15, 15> F_expected;
    F_expected.setIdentity();
    // Add expected values based on the computation in compute_state_transition_jacobian
    // For example:
    F_expected.block<3, 3>(drone_state_estimation::POS, drone_state_estimation::VEL) = Eigen::Matrix3d::Identity() * dt;
    // Add more expected values as needed

    EXPECT_TRUE(node.F_.isApprox(F_expected, 1e-6));
}