// GTSAM-ready C++/Eigen snippets for on-manifold IMU preintegration
// Based on Forster et al. (2016)
// ---------------------------------------------------------------
// Assumptions:
//  - Eigen is available
//  - GTSAM is available
//  - Biases use gtsam::imuBias::ConstantBias
//  - Rotations use gtsam::Rot3 and quaternions
//  - Preintegration parameters follow gtsam::PreintegrationParams

#include <Eigen/Dense>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/PreintegratedImuMeasurements.h>
#include <gtsam/base/Matrix.h>

// ---------------------------------------------------------------
// Utility: Skew-symmetric matrix
inline Eigen::Matrix3d Skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S <<     0, -v.z(),  v.y(),
          v.z(),     0, -v.x(),
         -v.y(),  v.x(),     0;
    return S;
}

// ---------------------------------------------------------------
// Incremental preintegration step (Equation 35–39)
// ---------------------------------------------------------------
struct PreintegratedState {
    Eigen::Quaterniond dR;     // Delta rotation
    Eigen::Vector3d dV;        // Delta velocity
    Eigen::Vector3d dP;        // Delta position
    Eigen::Matrix<double, 9, 9> A; // Jacobian wrt IMU state
    Eigen::Matrix<double, 9, 6> B; // Jacobian wrt IMU biases
    double dt_sum = 0.0;
};

// Single IMU increment
void IntegrateMeasurement(
        PreintegratedState& S,
        const Eigen::Vector3d& measured_acc,
        const Eigen::Vector3d& measured_gyro,
        const gtsam::imuBias::ConstantBias& bias,
        double dt)
{
    // Unbiased signals
    Eigen::Vector3d w = measured_gyro - bias.gyroscope();
    Eigen::Vector3d a = measured_acc  - bias.accelerometer();

    // ---------------------------------------------------------------
    // 1. Update delta rotation (Eq. 35)
    // ---------------------------------------------------------------
    Eigen::Vector3d wdt = w * dt;
    double theta = wdt.norm();
    Eigen::Quaterniond dQ_inc(1, 0, 0, 0);

    if (theta > 1e-5) {
        Eigen::Vector3d axis = wdt.normalized();
        dQ_inc = Eigen::AngleAxisd(theta, axis);
    }

    S.dR = S.dR * dQ_inc;
    S.dR.normalize();

    // ---------------------------------------------------------------
    // 2. Update delta velocity (Eq. 36)
    // ---------------------------------------------------------------
    Eigen::Vector3d acc_world = S.dR * a;
    S.dV += acc_world * dt;

    // ---------------------------------------------------------------
    // 3. Update delta position (Eq. 37)
    // ---------------------------------------------------------------
    S.dP += S.dV * dt + 0.5 * acc_world * dt * dt;

    // ---------------------------------------------------------------
    // 4. Update Jacobians A, B (Eq. 41–46)
    // ---------------------------------------------------------------

    Eigen::Matrix3d R = S.dR.toRotationMatrix();
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 9, 9> F = Eigen::Matrix<double, 9, 9>::Identity();
    Eigen::Matrix<double, 9, 6> G = Eigen::Matrix<double, 9, 6>::Zero();

    // F blocks
    F.block<3,3>(0,0) = I - Skew(w) * dt;            // dR wrt R
    F.block<3,3>(3,0) = -R * Skew(a) * dt;           // dV wrt R
    F.block<3,3>(6,0) = -0.5 * R * Skew(a) * dt*dt;  // dP wrt R
    F.block<3,3>(3,3) = I;                           // dV wrt V
    F.block<3,3>(6,3) = I * dt;                      // dP wrt V
    F.block<3,3>(6,6) = I;                           // dP wrt P

    // G blocks
    G.block<3,3>(0,3) = -I * dt;                     // dR wrt gyro bias
    G.block<3,3>(3,0) = -R * dt;                     // dV wrt acc bias
    G.block<3,3>(6,0) = -0.5 * R * dt * dt;          // dP wrt acc bias

    // Propagate Jacobians
    S.A = F * S.A;
    S.B = F * S.B + G;

    S.dt_sum += dt;
}

// ---------------------------------------------------------------
// Convert to GTSAM's PreintegratedImuMeasurements
// ---------------------------------------------------------------

std::shared_ptr<gtsam::PreintegratedImuMeasurements> ToGTSAM(
    const PreintegratedState& S,
    const std::shared_ptr<gtsam::PreintegrationParams>& params,
    const gtsam::imuBias::ConstantBias& bias)
{
    auto pim = std::make_shared<gtsam::PreintegratedImuMeasurements>(params, bias);

    // Set deltas
    pim->deltaRij() = gtsam::Rot3(S.dR);
    pim->deltaPij() = S.dP;
    pim->deltaVij() = S.dV;

    // Set Jacobians
    pim->setPreintegratedMeasurements( S.dP, S.dV, gtsam::Rot3(S.dR) );

    // Set time
    pim->deltaTij() = S.dt_sum;

    return pim;
}

// ---------------------------------------------------------------
// Example usage
// ---------------------------------------------------------------

void Example() {
    auto params = gtsam::PreintegrationParams::MakeSharedU(9.81);
    gtsam::imuBias::ConstantBias bias;

    PreintegratedState S;
    S.dR = Eigen::Quaterniond::Identity();
    S.dV.setZero();
    S.dP.setZero();
    S.A.setIdentity();
    S.B.setZero();

    double dt = 0.01;
    Eigen::Vector3d acc(0,0,0);
    Eigen::Vector3d gyro(0,0,0);

    // Integrate a few measurements
    for(int i=0;i<100;i++) {
        IntegrateMeasurement(S, acc, gyro, bias, dt);
    }

    auto pim = ToGTSAM(S, params, bias);
}
