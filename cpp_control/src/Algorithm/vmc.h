#pragma once

// This header is independent from MuJoCo.
// main.cpp and Python ctypes both call into the same VMC implementation.

struct RobotObservation {
    double time;

    // IMU quaternion from MuJoCo framequat sensor, order: w, x, y, z.
    double quat[4];

    // Euler angles: roll, pitch, yaw, unit: rad.
    double euler[3];

    // Gyro angular velocity: x, y, z, unit: rad/s.
    double gyro[3];

    // Active leg joint positions:
    // [right_front jAB, right_rear jAG, left_front jIJ, left_rear jIO]
    double joint_pos[4];

    // Wheel position and estimated velocity: [right, left].
    double wheel_pos[2];
    double wheel_vel[2];
};

struct TorqueCommand {
    // MuJoCo data->ctrl order:
    // [right_front, right_rear, left_front, left_rear, right_wheel, left_wheel]
    double ctrl[6];
};

void torque_zero(TorqueCommand* out);
void vmc_compute(const RobotObservation* obs, double dt, TorqueCommand* out);
void lqr_compute(const RobotObservation* obs, const TorqueCommand* vmc_out, TorqueCommand* out);

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _WIN32
#define VMC_API __declspec(dllexport)
#else
#define VMC_API
#endif

// Plain C ABI for Python ctypes.
VMC_API void vmc_compute_c(double time,
                           const double quat[4],
                           const double euler[3],
                           const double gyro[3],
                           const double joint_pos[4],
                           const double wheel_pos[2],
                           const double wheel_vel[2],
                           double dt,
                           double out_ctrl[6]);

#ifdef __cplusplus
}
#endif
