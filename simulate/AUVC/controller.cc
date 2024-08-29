#include "controller.h"
#include "mujoco/mjmodel.h"

#include <dlfcn.h>
#include <mujoco/mujoco.h>
#include <stdio.h>

static double Qorn[4] = {0}; // w,x,y,z
static double orn[3] = {
    0}; // r[forward axis],p[possibly side axis],y[vertical axis]
static double zOrn[3] = {0}; // Zero orientation

// a1,a2,a3,a4,a5,a6
const static float kp = 1.5;
const static float ki = 0.00005;
const static float kd = 0.00005;

static float error = 0;
static float last_error = 0;
static float intg = 0;
static float diff = 0;
static float prop = 0;
static float balance = 0;

mjtNum pid(mjtNum target, mjtNum currOrn);
void orn_Control(mjData *d, mjtNum target[3], mjtNum direction[3]);

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

extern "C" bool controllerUpdatePlug(mjModel *m, mjData *d) {

  // speed up props!
  float speed = d->time < 10.0 ? d->time * 2 * 10 : 200.0;
  // printf("speed: %f", speed);

  d->ctrl[0] = 0.0;   // fr_1_act
  d->ctrl[1] = 0.0;   // fr_2_act
  d->ctrl[2] = 0.0;   // fr_3_act
  d->ctrl[3] = -speed; // fr_prop force
  d->ctrl[4] = speed; // fr_prop

  d->ctrl[5] = 0.0;   // fl_1_act
  d->ctrl[6] = 0.0;   // fl_2_act
  d->ctrl[7] = 0.0;   // fl_3_act
  d->ctrl[8] = speed; // fl_prop force
  d->ctrl[9] = speed; // fl_prop

  d->ctrl[10] = 0.0;    // br_1_act
  d->ctrl[11] = 0.0;    // br_2_act
  d->ctrl[12] = 0.0;   // br_3_act
  d->ctrl[13] = speed; // br_prop force
  d->ctrl[14] = speed; // br_prop

  d->ctrl[15] = 0.0;   // bl_1_act
  d->ctrl[16] = 0.0;   // bl_2_act
  d->ctrl[17] = 0.0;   // bl_3_act
  d->ctrl[18] = -speed; // bl_prop force
  d->ctrl[19] = speed; // bl_prop
  return true;
}
