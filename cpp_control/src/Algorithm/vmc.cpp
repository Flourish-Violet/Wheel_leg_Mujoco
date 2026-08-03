#include "vmc.h"

#include <math.h>

namespace {

static const double kPi = 3.14159265358979323846;

typedef struct {
    double l1;
    double l2;
    double l3;
    double l4;
    double l5;

    double last_phi0;
    double last_L0;
    double last_d_L0;
    double last_d_theta;
    int first_flag;

    double ref_L0;
    double ref_theta;
} LegVmcState;

typedef struct {
    double phi1;
    double phi4;
    double phi2;
    double phi3;
    double phi0;
    double L0;
    double theta;
    double d_L0;
    double d_theta;
    double F0;
    double Tp;
} LegVmcKinematics;

static double clamp_value(double value, double lo, double hi) {
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

static void leg_state_init(LegVmcState* s) {
    s->l1 = 0.215;
    s->l2 = 0.258;
    s->l3 = 0.258;
    s->l4 = 0.215;
    s->l5 = 0.0;

    s->last_phi0 = 0.0;
    s->last_L0 = 0.0;
    s->last_d_L0 = 0.0;
    s->last_d_theta = 0.0;
    s->first_flag = 0;

    s->ref_L0 = 0.0;
    s->ref_theta = 0.0;
}

static void leg_kinematics_update(const LegVmcState* s,
                                  double phi1,
                                  double phi4,
                                  double pitch,
                                  double gyro,
                                  double dt,
                                  LegVmcKinematics* k) {
    const double PitchR = -pitch;
    const double GyroR = -gyro;

    const double YB = s->l1 * sin(phi1);
    const double XB = s->l1 * cos(phi1);
    const double YD = s->l4 * sin(phi4);
    const double XD = s->l5 + s->l4 * cos(phi4);

    const double lBD = sqrt((XD - XB) * (XD - XB) + (YD - YB) * (YD - YB));
    const double A0 = 2.0 * s->l2 * (XD - XB);
    const double B0 = 2.0 * s->l2 * (YD - YB);
    const double C0 = s->l2 * s->l2 + lBD * lBD - s->l3 * s->l3;

    double discriminant = A0 * A0 + B0 * B0 - C0 * C0;
    if (discriminant < 0.0) {
        discriminant = 0.0;
    }

    const double numerator = B0 + sqrt(discriminant);
    const double denominator = A0 + C0;

    double phi2 = 0.0;
    if (fabs(denominator) > 1e-12) {
        phi2 = 2.0 * atan2(numerator, denominator);
    }

    const double dy = YB - YD + s->l2 * sin(phi2);
    const double dx = XB - XD + s->l2 * cos(phi2);
    const double phi3 = atan2(dy, dx);

    const double XC = XB + s->l2 * cos(phi2);
    const double YC = YB + s->l2 * sin(phi2);

    const double phi0 = atan2(YC, (XC - s->l5 / 2.0));
    const double L0 = sqrt((XC - s->l5 / 2.0) * (XC - s->l5 / 2.0) + YC * YC);

    double d_phi0 = 0.0;
    if (s->first_flag == 0) {
        d_phi0 = 0.0;
    } else {
        d_phi0 = (phi0 - s->last_phi0) / dt;
    }

    const double theta = kPi / 2.0 - PitchR - phi0;
    const double d_theta = -GyroR - d_phi0;

    k->phi1 = phi1;
    k->phi4 = phi4;
    k->phi2 = phi2;
    k->phi3 = phi3;
    k->phi0 = phi0;
    k->L0 = L0;
    k->theta = theta;
    k->d_L0 = (s->first_flag == 0) ? 0.0 : (L0 - s->last_L0) / dt;
    k->d_theta = d_theta;
    k->F0 = 0.0;
    k->Tp = 0.0;
}

static void leg_control_init_reference(LegVmcState* s, const LegVmcKinematics* k) {
    if (s->first_flag == 0) {
        s->ref_L0 = k->L0 + 0.04;
        s->ref_theta = k->theta;
        s->first_flag = 1;
    }
}

static void leg_control_update_state(LegVmcState* s, const LegVmcKinematics* k) {
    s->last_phi0 = k->phi0;
    s->last_L0 = k->L0;
    s->last_d_L0 = k->d_L0;
    s->last_d_theta = k->d_theta;
}

static void leg_compute_torque(LegVmcState* s, const LegVmcKinematics* k, TorqueCommand* out, int offset) {
    const double kp_L = 80.0;
    const double kd_L = 4.0;
    const double kp_theta = 8.0;
    const double kd_theta = 0.5;

    const double err_L = s->ref_L0 - k->L0;
    const double err_theta = s->ref_theta - k->theta;
    const double err_dL = 0.0 - k->d_L0;
    const double err_dtheta = 0.0 - k->d_theta;

    double F0 = kp_L * err_L + kd_L * err_dL;
    double Tp = kp_theta * err_theta + kd_theta * err_dtheta;

    F0 = clamp_value(F0, -50.0, 50.0);
    Tp = clamp_value(Tp, -8.0, 8.0);

    const double sin_phi3_phi2 = sin(k->phi3 - k->phi2);
    if (fabs(sin_phi3_phi2) < 1e-8 || fabs(k->L0) < 1e-8) {
        out->ctrl[offset + 0] = 0.0;
        out->ctrl[offset + 1] = 0.0;
        return;
    }

    const double j11 = (s->l1 * sin(k->phi0 - k->phi3) * sin(k->phi1 - k->phi2)) / sin_phi3_phi2;
    const double j12 = (s->l1 * cos(k->phi0 - k->phi3) * sin(k->phi1 - k->phi2)) / (k->L0 * sin_phi3_phi2);
    const double j21 = (s->l4 * sin(k->phi0 - k->phi2) * sin(k->phi3 - k->phi4)) / sin_phi3_phi2;
    const double j22 = (s->l4 * cos(k->phi0 - k->phi2) * sin(k->phi3 - k->phi4)) / (k->L0 * sin_phi3_phi2);

    out->ctrl[offset + 0] = j21 * F0 + j22 * Tp;
    out->ctrl[offset + 1] = j11 * F0 + j12 * Tp;
}

static void leg_vmc_step(LegVmcState* s,
                         double phi1,
                         double phi4,
                         double pitch,
                         double gyro,
                         double dt,
                         TorqueCommand* out,
                         int offset) {
    LegVmcKinematics k;
    leg_kinematics_update(s, phi1, phi4, pitch, gyro, dt, &k);
    leg_control_init_reference(s, &k);
    leg_control_update_state(s, &k);
    leg_compute_torque(s, &k, out, offset);
}

}  // namespace

void torque_zero(TorqueCommand* out) {
    for (int i = 0; i < 6; ++i) {
        out->ctrl[i] = 0.0;
    }
}

void vmc_compute(const RobotObservation* obs, double dt, TorqueCommand* out) {
    static LegVmcState right_state;
    static LegVmcState left_state;
    static int initialized = 0;

    if (initialized == 0) {
        leg_state_init(&right_state);
        leg_state_init(&left_state);
        initialized = 1;
    }

    torque_zero(out);

    // 参照 Python 版的调用方式：
    // right: phi1 = joint_pos[0] + pi, phi4 = joint_pos[1], pitch = +pitch, gyro = +gyro_y
    // left : phi1 = joint_pos[3] + pi, phi4 = joint_pos[2], pitch = -pitch, gyro = -gyro_y
    //
    // 这里先做一个轻量的 VMC 闭环：
    // 1. 通过当前姿态和腿长算出 F0 / Tp
    // 2. 再把 F0 / Tp 映射成两个关节力矩
    // 3. 轮子暂时保持 0，后面你可以在这里接速度环或 LQR
    leg_vmc_step(&right_state,
                 obs->joint_pos[0] + kPi,
                 obs->joint_pos[1],
                 obs->euler[1],
                 obs->gyro[1],
                 dt,
                 out,
                 0);

    leg_vmc_step(&left_state,
                 obs->joint_pos[3] + kPi,
                 obs->joint_pos[2],
                 -obs->euler[1],
                 -obs->gyro[1],
                 dt,
                 out,
                 2);

    out->ctrl[4] = 0.0;
    out->ctrl[5] = 0.0;
}

void lqr_compute(const RobotObservation* obs, const TorqueCommand* vmc_out, TorqueCommand* out) {
    (void)obs;
    *out = *vmc_out;
}

void vmc_compute_c(double time,
                   const double quat[4],
                   const double euler[3],
                   const double gyro[3],
                   const double joint_pos[4],
                   const double wheel_pos[2],
                   const double wheel_vel[2],
                   double dt,
                   double out_ctrl[6]) {
    RobotObservation obs;
    obs.time = time;
    for (int i = 0; i < 4; ++i) {
        obs.quat[i] = quat[i];
        obs.joint_pos[i] = joint_pos[i];
    }
    for (int i = 0; i < 3; ++i) {
        obs.euler[i] = euler[i];
        obs.gyro[i] = gyro[i];
    }
    for (int i = 0; i < 2; ++i) {
        obs.wheel_pos[i] = wheel_pos[i];
        obs.wheel_vel[i] = wheel_vel[i];
    }

    TorqueCommand vmc_out;
    TorqueCommand final_out;
    vmc_compute(&obs, dt, &vmc_out);
    lqr_compute(&obs, &vmc_out, &final_out);

    for (int i = 0; i < 6; ++i) {
        out_ctrl[i] = final_out.ctrl[i];
    }
}
