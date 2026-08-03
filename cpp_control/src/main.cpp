#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <mujoco/mujoco.h>

#include "Algorithm/vmc.h"
#include "viewer_glfw.h"

#define PI_VALUE 3.14159265358979323846

static const char* kModelPath = "E:/Mujoco/wheel_leg_mujoco/MJCF/env.xml";
static const int kControlDiv = 4;  // MuJoCo timestep=1ms，所以这里是 4ms 控制一次。

static int sensor_id(const mjModel* m, const char* name) {
    int id = mj_name2id(m, mjOBJ_SENSOR, name);
    if (id < 0) {
        printf("找不到 sensor: %s\n", name);
    }
    return id;
}

static double sensor_scalar(const mjModel* m, const mjData* d, int id) {
    // MuJoCo 把所有 sensor 数据放在 data->sensordata 这一维数组里。
    // model->sensor_adr[id] 是这个 sensor 在 sensordata 里的起始下标。
    return d->sensordata[m->sensor_adr[id]];
}

static void sensor_vec(const mjModel* m, const mjData* d, int id, double* out, int n) {
    // 对于 gyro 这种 3 维 sensor：
    // adr = model->sensor_adr[id]
    // sensordata[adr + 0/1/2] 分别是 x/y/z。
    int adr = m->sensor_adr[id];
    for (int i = 0; i < n; ++i) {
        out[i] = d->sensordata[adr + i];
    }
}

static void quat_to_euler(const double q[4], double euler[3]) {
    double w = q[0], x = q[1], y = q[2], z = q[3];
    double norm = sqrt(w * w + x * x + y * y + z * z);
    if (norm > 1e-12) {
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
    }

    euler[0] = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));

    double sinp = 2.0 * (w * y - z * x);
    if (fabs(sinp) >= 1.0) {
        euler[1] = copysign(PI_VALUE / 2.0, sinp);
    } else {
        euler[1] = asin(sinp);
    }

    euler[2] = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

struct SensorMap {
    int orientation;
    int gyro;
    int right_wheel_pos;
    int left_wheel_pos;
    int right_front_joint_pos;
    int right_rear_joint_pos;
    int left_front_joint_pos;
    int left_rear_joint_pos;
    double last_wheel_pos[2];
};

static int init_sensors(const mjModel* m, SensorMap* s) {
    s->orientation = sensor_id(m, "orientation");
    s->gyro = sensor_id(m, "gyro");
    s->right_wheel_pos = sensor_id(m, "Right_Wheel_pos");
    s->left_wheel_pos = sensor_id(m, "Left_Wheel_pos");
    s->right_front_joint_pos = sensor_id(m, "Right_front_joint_pos");
    s->right_rear_joint_pos = sensor_id(m, "Right_rear_joint_pos");
    s->left_front_joint_pos = sensor_id(m, "Left_front_joint_pos");
    s->left_rear_joint_pos = sensor_id(m, "Left_rear_joint_pos");
    s->last_wheel_pos[0] = 0.0;
    s->last_wheel_pos[1] = 0.0;

    return s->orientation >= 0 && s->gyro >= 0 &&
           s->right_wheel_pos >= 0 && s->left_wheel_pos >= 0 &&
           s->right_front_joint_pos >= 0 && s->right_rear_joint_pos >= 0 &&
           s->left_front_joint_pos >= 0 && s->left_rear_joint_pos >= 0;
}

static void read_observation(const mjModel* m, const mjData* d, SensorMap* s, RobotObservation* obs) {
    obs->time = d->time;

    // orientation 是 4 维 framequat sensor，顺序是 wxyz。
    sensor_vec(m, d, s->orientation, obs->quat, 4);
    quat_to_euler(obs->quat, obs->euler);

    // gyro 是 3 维 sensor，对应机体角速度。
    sensor_vec(m, d, s->gyro, obs->gyro, 3);

    obs->wheel_pos[0] = sensor_scalar(m, d, s->right_wheel_pos);
    obs->wheel_pos[1] = sensor_scalar(m, d, s->left_wheel_pos);
    obs->wheel_vel[0] = (obs->wheel_pos[0] - s->last_wheel_pos[0]) / m->opt.timestep;
    obs->wheel_vel[1] = -(obs->wheel_pos[1] - s->last_wheel_pos[1]) / m->opt.timestep;
    s->last_wheel_pos[0] = obs->wheel_pos[0];
    s->last_wheel_pos[1] = obs->wheel_pos[1];

    // 这里的零点偏置和 Python environment.py 保持一致。
    obs->joint_pos[0] = sensor_scalar(m, d, s->right_front_joint_pos) + 0.027;
    obs->joint_pos[1] = sensor_scalar(m, d, s->right_rear_joint_pos) + 1.3;
    obs->joint_pos[2] = sensor_scalar(m, d, s->left_front_joint_pos) + 0.003;
    obs->joint_pos[3] = sensor_scalar(m, d, s->left_rear_joint_pos) - 1.3;
}

static double clamp_ctrl(double x, double lo, double hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static void apply_ctrl(const mjModel* m, mjData* d, const TorqueCommand* cmd) {
    for (int i = 0; i < m->nu && i < 6; ++i) {
        double lo = m->actuator_ctrlrange[2 * i + 0];
        double hi = m->actuator_ctrlrange[2 * i + 1];
        d->ctrl[i] = clamp_ctrl(cmd->ctrl[i], lo, hi);
    }
}

static void print_actuator_map(const mjModel* m) {
    printf("nu=%d, nsensor=%d, timestep=%.6f\n", m->nu, m->nsensor, m->opt.timestep);
    for (int i = 0; i < m->nu; ++i) {
        const char* name = mj_id2name(m, mjOBJ_ACTUATOR, i);
        printf("ctrl[%d] -> %s, range=[%.1f, %.1f]\n",
               i, name ? name : "null",
               m->actuator_ctrlrange[2 * i + 0],
               m->actuator_ctrlrange[2 * i + 1]);
    }
}

struct ControlContext {
    SensorMap sensors;
    RobotObservation obs;
    TorqueCommand vmc_out;
    TorqueCommand final_out;
    int step;
};

static void control_step(const mjModel* m, mjData* d, void* user) {
    ControlContext* ctx = (ControlContext*)user;

    read_observation(m, d, &ctx->sensors, &ctx->obs);

    if (ctx->step % kControlDiv == 0) {
        double control_dt = m->opt.timestep * kControlDiv;
        vmc_compute(&ctx->obs, control_dt, &ctx->vmc_out);
        lqr_compute(&ctx->obs, &ctx->vmc_out, &ctx->final_out);
        apply_ctrl(m, d, &ctx->final_out);
    }

    if (ctx->step % 1000 == 0) {
        printf("t=%.3f pitch=%.6f ctrl=[%.2f %.2f %.2f %.2f %.2f %.2f]\n",
               d->time, ctx->obs.euler[1],
               d->ctrl[0], d->ctrl[1], d->ctrl[2],
               d->ctrl[3], d->ctrl[4], d->ctrl[5]);
    }

    ctx->step += 1;
}

int main(void) {
    char error[1024] = {0};
    mjModel* m = mj_loadXML(kModelPath, NULL, error, sizeof(error));
    if (!m) {
        printf("Model load Failed: %s\n", error);
        return EXIT_FAILURE;
    }

    mjData* d = mj_makeData(m);
    if (!d) {
        printf("mj_makeData 失败\n");
        mj_deleteModel(m);
        return EXIT_FAILURE;
    }

    printf("Model Load Successfully: %s\n", kModelPath);
    print_actuator_map(m);

    ControlContext ctx;
    if (!init_sensors(m, &ctx.sensors)) {
        mj_deleteData(d);
        mj_deleteModel(m);
        return EXIT_FAILURE;
    }

    torque_zero(&ctx.vmc_out);
    torque_zero(&ctx.final_out);
    ctx.step = 0;

    mj_forward(m, d);
    int viewer_status = run_mujoco_viewer(m, d, control_step, &ctx);

    mj_deleteData(d);
    mj_deleteModel(m);
    return viewer_status == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
