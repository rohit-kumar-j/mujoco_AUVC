#include "controller.h"
#include "mujoco/mjmodel.h"

#include <dlfcn.h>
#include <math.h>
#include <mujoco/mujoco.h>
#include <stdio.h>

static float theta = 0;
void quaternion_to_euler(double x, double y, double z, double w, double *X,
                         double *Y, double *Z) {
  double t0 = +2.0 * (w * x + y * z);
  double t1 = +1.0 - 2.0 * (x * x + y * y);
  *X = atan2(t0, t1) * (180.0 / M_PI);

  double t2 = +2.0 * (w * y - z * x);
  if (t2 > 1)
    t2 = 1;
  else
    t2 = t2;
  if (t2 < -1)
    t2 = -1;
  else
    t2 = t2;
  *Y = asin(t2) * (180.0 / M_PI);

  double t3 = +2.0 * (w * z + x * y);
  double t4 = +1.0 - 2.0 * (y * y + z * z);
  *Z = atan2(t3, t4) * (180.0 / M_PI);
}

extern "C" void controllerTestFunc(void) { printf("Hi from Plugin!\n"); }

extern "C" int controllerInitPlug(mjModel *m, mjData *d) {
  printf("Init Plug!\n");
  // // Store Zero State data
  // for(int i=0; i<4; i++){
  //   Qorn[i] = d->sensordata[i];
  // }
  // quaternion_to_euler(Qorn[1], Qorn[2], Qorn[3], Qorn[0], &zOrn[0], &zOrn[1],
  // &zOrn[2]); printf("zOrn: [%f, %f, %f]", zOrn[0],zOrn[1],zOrn[2]);

  // Set this as the current baseline!
  // to set new Orientation, we simply say target[yaw] = 10;
  // i.e move yaw to zOrn[yaw] + currOrn[yaw] +  10:

  return 0;
}

float clip(float val, float min, float max) {
  if (val >= max)
    val = max;
  else if (val <= min)
    val = min;
  return val;
}

void set_props_speed(mjData *d, float speed);
void set_hinge_joints_zero(mjData *d);
void single_leg_pattern(mjData *d);
void fold_legs(mjData *d);
void drive_config_1(mjData *d);
void drive_with_props_1(mjData *d, float speed);

extern "C" bool controllerUpdatePlug(mjModel *m, mjData *d) {

  // Walking section!
  // set_props_speed(d, 0);
  // single_leg_pattern(d);

  // Flying section!
  // float speed = d->time < 10.0 ? d->time * 17 * 10 : 1700.0;
  // printf("speed: %f\n", speed);
  // set_hinge_joints_zero(d);
  // set_props_speed(d,speed);

  // Flying with legs closed!
  // float speed = d->time < 10.0 ? d->time * 17 * 10 : 1700.0;
  // fold_legs(d);
  // set_props_speed(d, speed);

  // Driving configuration 1
  float speed = d->time < 10.0 ? d->time * 1 * 10 : 100.0;
  drive_config_1(d);
  drive_with_props_1(d, speed);

  return true;
}

void set_hinge_joints_zero(mjData *d) {

  d->ctrl[0] = 0.0;  // fr_1_act
  d->ctrl[1] = 0.0;  // fr_2_act
  d->ctrl[2] = 0.0;  // fr_3_act
  d->ctrl[5] = 0.0;  // fl_1_act
  d->ctrl[6] = 0.0;  // fl_2_act
  d->ctrl[7] = 0.0;  // fl_3_act
  d->ctrl[10] = 0.0; // br_1_act
  d->ctrl[11] = 0.0; // br_2_act
  d->ctrl[12] = 0.0; // br_3_act
  d->ctrl[15] = 0.0; // bl_1_act
  d->ctrl[16] = 0.0; // bl_2_act
  d->ctrl[17] = 0.0; // bl_3_act
}

void fold_legs(mjData *d) {
  d->ctrl[0] = 0.0;
  d->ctrl[1] = 0.0;
  d->ctrl[2] = 70.0;

  d->ctrl[5] = 0.0;
  d->ctrl[6] = 0.0;
  d->ctrl[7] = 70.0;

  d->ctrl[10] = 0.0;
  d->ctrl[11] = 0.0;
  d->ctrl[12] = 70.0;

  d->ctrl[15] = 0.0;
  d->ctrl[16] = 0.0;
  d->ctrl[17] = 70.0;
}

void set_props_speed(mjData *d, float speed) {
  d->ctrl[3] = -speed;  // fr_prop force
  d->ctrl[4] = speed;   // fr_prop
  d->ctrl[8] = speed;   // fl_prop force
  d->ctrl[9] = speed;   // fl_prop
  d->ctrl[13] = speed;  // br_prop force
  d->ctrl[14] = speed;  // br_prop
  d->ctrl[18] = -speed; // bl_prop force
  d->ctrl[19] = speed;  // bl_prop
}

void single_leg_pattern(mjData *d) {
  float target_hip = 8;
  float target_knee = 15;
  float target_foot = -25;
  int phase = int(d->time) % 2;

  // printf("int(d->time) \% 2 : %d\n", phase);

  int mul = 1;
  // printf("phase: %d\n", phase);
  if (phase) {
    mul = -1;
  }
  theta += 0.002;
  float dt = (5.0 * M_PI) / 2.0;
  if (0 <= theta && theta <= dt) {
    printf("p1: %f/ %f \n", theta, dt);
    d->ctrl[0] = sin(theta) * -target_hip;                           // fr_1_act
    d->ctrl[1] = clip(sin(theta - 0.7) * target_knee, -2.0, 4.0);    // fr_2_act
    d->ctrl[2] = clip(sin(theta - 0.7) * target_foot, -15.0, -10.0); // fr_3_act
  }
  if (dt <= theta && theta <= 2 * dt) {
    printf("p2: %f/ %f \n", theta, 2 * dt);
    d->ctrl[15] = sin(theta) * target_hip;                          // bl_1_act
    d->ctrl[16] = clip(sin(theta - 0.7) * -target_knee, -2.0, 4.0); // bl_2_act
    d->ctrl[17] =
        clip(sin(theta - 0.7) * -target_foot, -15.0, -10.0); // bl_3_act
  }
  if (2 * dt <= theta && theta <= 3 * dt) {
    printf("p3: %f/ %f \n", theta, 3 * dt);
    d->ctrl[5] = sin(theta) * -target_hip;                          // fl_1_act
    d->ctrl[6] = clip(sin(theta - 0.7) * target_knee, -7.0, 0.0);   // fl_2_act
    d->ctrl[7] = clip(sin(theta - 0.7) * -target_foot, -15.0, 0.0); // fl_3_act
  }
  if (3 * dt <= theta && theta <= 4 * dt) {
    printf("p4: %f/ %f \n", theta, 4 * dt);
    d->ctrl[10] = sin(theta) * -target_hip;                          // br_1_act
    d->ctrl[11] = clip(sin(theta - 0.7) * target_knee, -7.0, 0.0);   // br_2_act
    d->ctrl[12] = clip(sin(theta - 0.7) * -target_foot, -15.0, 0.0); // br_2_act
  }
  if (theta >= 4 * dt)
    theta = 0;

  // if(0 <= theta && theta <= M_PI){
  //   d->ctrl[0] = sin(theta) * -target_hip;                           //
  //   fr_1_act d->ctrl[1] = clip(sin(theta - 0.7) * target_knee, -2.0, 4.0); //
  //   fr_2_act d->ctrl[2] = clip(sin(theta - 0.7) * target_foot, -15.0, -10.0);
  //   // fr_3_act
  // }
  // if(M_PI <= theta && theta <= 2.0 * M_PI){
  //   d->ctrl[15] = sin(theta+ M_PI) * target_hip; // bl_1_act d->ctrl[16] =
  //   clip(sin(theta + M_PI - 0.7) * -target_knee, -2.0, 4.0);    // fr_2_act
  //   d->ctrl[17] = clip(sin(theta + M_PI - 0.7) * -target_foot, -15.0, -10.0);
  //   //fr_3_act
  // }
  // if(M_PI <= theta && theta <= 2.0 * M_PI){
  //   d->ctrl[5] = sin(theta + 2.0*M_PI ) * -target_hip; // fl_1_act d->ctrl[6]
  //   = clip(sin(theta + 2.0*M_PI  - 0.7) * target_knee, -7.0, 0.0);   //
  //   fl_2_act d->ctrl[7] = clip(sin(theta + 2.0*M_PI  - 0.7) * -target_foot,
  //   -15.0, 0.0); // fl_2_act
  // }
  // //
  // if(M_PI <= theta && theta <= 3.0 * M_PI){
  //   d->ctrl[10] = sin(theta + 3.0* M_PI) * -target_hip; // br_1_act
  //   d->ctrl[11] = clip(sin(theta + 3.0* M_PI - 0.7) * target_knee, -7.0,
  //   0.0);   // br_2_act d->ctrl[12] = clip(sin(theta + 3.0* M_PI - 0.7) *
  //   -target_foot, -15.0, 0.0); // br_2_act
  // }
  // printf("clipped: %f\n", d->ctrl[12]);                              //
  // fr_2_act

  // printf("ctrl[0] : %f | 15: %f | theta : %f\n", d->ctrl[0], d->ctrl[15],
  //        theta);

  // printf("theta : %f | sdata : %f\n", theta, d->sensordata[0]);

  //
  //
  //
  // d->ctrl[10] = mul * -target_hip; // br_1_act
  // d->ctrl[11] = mul * target_knee; // br_2_act
  // d->ctrl[12] = mul * target_foot; // br_3_act

  // d->ctrl[0] = mul * -target_hip;  // fr_1_act
  // d->ctrl[1] = mul * target_knee;  // fr_2_act
  // d->ctrl[2] = mul * target_foot;  // fr_3_act
  // d->ctrl[15] = mul * -target_hip; // bl_1_act
  // d->ctrl[16] = mul * target_knee; // bl_2_act
  // d->ctrl[17] = mul * target_foot; // bl_3_act
  // printf("[fr: %f %f %f]\n", d->sensordata[0],
  //        d->sensordata[1] * (180.0 / M_PI), d->sensordata[2] * (180.0 /
  //        M_PI));
  // printf("[fl: %f %f %f]\n", d->sensordata[3] * (180.0 / M_PI),
  //        d->sensordata[4] * (180.0 / M_PI), d->sensordata[5] * (180.0 /
  //        M_PI));
  // printf("[br: %f %f %f]\n", d->sensordata[6] * (180.0 / M_PI),
  //        d->sensordata[7] * (180.0 / M_PI), d->sensordata[8] * (180.0 /
  //        M_PI));
  // printf("[bl: %f %f %f]\n\n", d->sensordata[9] * (180.0 / M_PI),
  //        d->sensordata[10] * (180.0 / M_PI),
  //        d->sensordata[11] * (180.0 / M_PI));

  // printf("[fr_hip: %f\n", d->sensordata[0] * (180.0/M_PI));
  // printf("fr_knee: %f\n", d->sensordata[1] * (180.0/M_PI));
  // printf("fr_foot: %f\n", d->sensordata[2] * (180.0/M_PI));
  //
  // printf("fl_hip: %f\n", d->sensordata[3] * (180.0/M_PI));
  // printf("fl_knee: %f\n", d->sensordata[4] * (180.0/M_PI));
  // printf("fl_foot: %f\n", d->sensordata[5] * (180.0/M_PI));
  //
  // printf("br_hip: %f\n", d->sensordata[6] * (180.0/M_PI));
  // printf("br_knee: %f\n", d->sensordata[7] * (180.0/M_PI));
  // printf("br_foot: %f\n", d->sensordata[8] * (180.0/M_PI));
  //
  // printf("bl_hip: %f\n", d->sensordata[9] * (180.0/M_PI));
  // printf("bl_knee: %f\n", d->sensordata[10] * (180.0/M_PI));
  // printf("bl_foot: %f\n", d->sensordata[11] * (180.0/M_PI));
}

void drive_config_1(mjData *d) {
  d->ctrl[0] = -45;
  d->ctrl[1] = -90.0;
  d->ctrl[2] = 0.1;

  d->ctrl[5] = 45;
  d->ctrl[6] = 90.0;
  d->ctrl[7] = 0.1;

  d->ctrl[10] = 45;
  d->ctrl[11] = 90.0;
  d->ctrl[12] = 0.1;

  d->ctrl[15] = -45;
  d->ctrl[16] = 90.0;
  d->ctrl[17] = 0.1;
}

void drive_with_props_1(mjData *d, float speed) {
  d->ctrl[3] = -speed; // fr_prop force
  // d->ctrl[4] = speed;   // fr_prop
  d->ctrl[8] = speed; // fl_prop force
  // d->ctrl[9] = speed;   // fl_prop
  d->ctrl[13] = -speed; // br_prop force
  // d->ctrl[14] = -speed; // br_prop
  d->ctrl[18] = speed; // bl_prop force
  // d->ctrl[19] = speed;  // bl_prop
}
