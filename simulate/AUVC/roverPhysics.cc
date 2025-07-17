#include "roverPhysics.h"

#include <dlfcn.h>
#include <mujoco/mujoco.h>
#include <stdio.h>

static double _Qorn[4] = {0}; // w,x,y,z
static double _orn[3] = {0}; // r[forward axis],p[possibly side axis],y[vertical axis]

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

extern "C" void roverPhysicsTestPlug(void) { printf("Hi from Rover!\n"); }

extern "C" int roverPhysicsInitPlug(mjModel *m, mjData *d) {
  printf("Init Rover Plug!\n");
  return 0;
}

// NICE
extern "C" bool roverPhysicsUpdatePlug(mjModel *m, mjData *d) {

  // Physical constants
  const float water_density = 1000.0f;  // kg/m³
  const float gravity = 9.806f;         // m/s²
  const float water_surface_height = 2.0f;  // Water surface level

  // Vehicle dimensions
  const float height = 0.254f;    // 10 inches - height of buoyancy volume

  // Get submarine mass and calculate gravitational force
  int body_id = mj_name2id(m, mjOBJ_BODY, "sub");
  float total_mass = (body_id >= 0) ? m->body_mass[body_id] : 9.0f;  // fallback to 9kg
  float total_gravity_force = total_mass * gravity;

  // Set buoyancy for slow sinking (adjust this ratio to control sink rate)
  const float buoyancy_ratio = 1.1f;  // 90% of gravity = slow sink
  float max_total_buoyancy = total_gravity_force * buoyancy_ratio;
  float max_buoyancy_per_corner = max_total_buoyancy / 4.0f;

  // Get vehicle vertical position (you mentioned qpos[2] is Y position)
  float vehicle_height = d->qpos[2];

  // Capture orientation quaternion
  for(int i = 0; i < 4; i++){
    _Qorn[i] = d->sensordata[i];
  }

  // Get site positions directly from MuJoCo data
  // Sites sfloat1-4 should correspond to site IDs 
  int site_ids[4];
  for(int i = 0; i < 4; i++) {
    char site_name[20];
    sprintf(site_name, "sfloat%d", i+1);
    site_ids[i] = mj_name2id(m, mjOBJ_SITE, site_name);
  }

  // Calculate buoyancy for each corner
  for(int i = 0; i < 4; i++) {
    float buoyancy_force = 0.0f;

    if(site_ids[i] >= 0) {
      // Get the Z-coordinate (vertical position) of this site
      float site_z = d->site_xpos[3 * site_ids[i] + 2];  // Z component

      // Calculate submersion depth
      float submersion_depth = water_surface_height - site_z;

      if(submersion_depth > 0.0f) {
        // Site is submerged
        // Clamp submersion to maximum effective depth
        float effective_depth = (submersion_depth > height) ? height : submersion_depth;

        // Buoyancy force proportional to submerged volume fraction
        float submerged_fraction = effective_depth / height;
        buoyancy_force = max_buoyancy_per_corner * submerged_fraction;
      }
    }

    // Apply purely vertical (upward) force
    d->ctrl[6 + i] = buoyancy_force;
  }

  return true;
}

extern "C" bool roverPhysicsUpdatePlugInteresting2(mjModel *m, mjData *d) {
  // Physical constants
  const float water_density = 1000.0f;  // kg/m³
  const float gravity = 9.806f;         // m/s²
  const float water_surface_height = 2.0f;  // Water surface level
  const float height = 0.254f;    // 10 inches - height of buoyancy volume

  // Drag coefficients (tune these for desired behavior)
  const float linear_drag_coeff = 5.0f;   // Linear drag coefficient
  const float angular_drag_coeff = 2.0f;  // Angular drag coefficient
  const float quadratic_drag_coeff = 0.5f; // Quadratic drag (for higher speeds)

  // Get submarine body
  int body_id = mj_name2id(m, mjOBJ_BODY, "sub");
  if(body_id < 0) return false;

  float total_mass = m->body_mass[body_id];
  float total_gravity_force = total_mass * gravity;

  // Set buoyancy for neutral buoyancy (adjust very precisely)
  const float buoyancy_ratio = 1.0f;  // 100% = neutral buoyancy
  float max_total_buoyancy = total_gravity_force * buoyancy_ratio;
  float max_buoyancy_per_corner = max_total_buoyancy / 4.0f;

  // Get vehicle state
  float vehicle_height = d->qpos[2];

  // Capture orientation quaternion
  for(int i = 0; i < 4; i++){
    _Qorn[i] = d->sensordata[i];
  }

  // Get site positions
  int site_ids[4];
  for(int i = 0; i < 4; i++) {
    char site_name[20];
    sprintf(site_name, "sfloat%d", i+1);
    site_ids[i] = mj_name2id(m, mjOBJ_SITE, site_name);
  }

  // Initialize forces
  float total_buoyancy_force = 0.0f;
  float total_torque[3] = {0.0f, 0.0f, 0.0f};

  // Get body COM position and velocity
  float body_com[3];
  float body_vel[3];
  float body_angvel[3];

  for(int i = 0; i < 3; i++) {
    body_com[i] = d->xpos[3 * body_id + i];
    body_vel[i] = d->cvel[6 * body_id + 3 + i];  // Linear velocity
    body_angvel[i] = d->cvel[6 * body_id + i];    // Angular velocity
  }

  // Calculate buoyancy for each corner
  for(int i = 0; i < 4; i++) {
    if(site_ids[i] >= 0) {
      float site_pos[3];
      for(int j = 0; j < 3; j++) {
        site_pos[j] = d->site_xpos[3 * site_ids[i] + j];
      }

      float submersion_depth = water_surface_height - site_pos[2];

      if(submersion_depth > 0.0f) {
        float effective_depth = (submersion_depth > height) ? height : submersion_depth;
        float submerged_fraction = effective_depth / height;
        float buoyancy_force = max_buoyancy_per_corner * submerged_fraction;

        total_buoyancy_force += buoyancy_force;

        // Calculate torque
        float r[3];
        for(int j = 0; j < 3; j++) {
          r[j] = site_pos[j] - body_com[j];
        }

        total_torque[0] += r[1] * buoyancy_force;
        total_torque[1] += -r[0] * buoyancy_force;
      }
    }
  }

  // Calculate drag forces
  float drag_force[3] = {0.0f, 0.0f, 0.0f};
  float drag_torque[3] = {0.0f, 0.0f, 0.0f};

  for(int i = 0; i < 3; i++) {
    // Linear drag: F_drag = -c * v - c_quad * v * |v|
    float vel_magnitude = sqrt(body_vel[0]*body_vel[0] + 
                               body_vel[1]*body_vel[1] + 
                               body_vel[2]*body_vel[2]);

    drag_force[i] = -linear_drag_coeff * body_vel[i] 
      - quadratic_drag_coeff * body_vel[i] * vel_magnitude;

    // Angular drag: T_drag = -c * omega
    drag_torque[i] = -angular_drag_coeff * body_angvel[i];
  }

  // Apply all forces and torques
  d->xfrc_applied[6 * body_id + 0] = drag_force[0];
  d->xfrc_applied[6 * body_id + 1] = drag_force[1];
  d->xfrc_applied[6 * body_id + 2] = total_buoyancy_force + drag_force[2];
  d->xfrc_applied[6 * body_id + 3] = total_torque[0] + drag_torque[0];
  d->xfrc_applied[6 * body_id + 4] = total_torque[1] + drag_torque[1];
  d->xfrc_applied[6 * body_id + 5] = total_torque[2] + drag_torque[2];

  // Optional: Add small disturbance forces for testing
  // This simulates water currents or control inputs
  if(false) {  // Set to true to enable
    float disturbance = 0.1f;  // Very small force
    d->xfrc_applied[6 * body_id + 0] += disturbance;
  }

  return true;
}

extern "C" bool roverPhysicsUpdatePlugInteresting(mjModel *m, mjData *d) {
  // Physical constants
  const float water_density = 1000.0f;  // kg/m³
  const float gravity = 9.806f;         // m/s²
  const float water_surface_height = 2.0f;  // Water surface level
  // Vehicle dimensions
  const float height = 0.254f;    // 10 inches - height of buoyancy volume

  // Get submarine mass and body ID
  int body_id = mj_name2id(m, mjOBJ_BODY, "sub");
  if(body_id < 0) return false;  // Body not found

  float total_mass = m->body_mass[body_id];
  float total_gravity_force = total_mass * gravity;

  // Set buoyancy for slow sinking
  const float buoyancy_ratio = 1.0f;  // 110% of gravity = slight positive buoyancy
  float max_total_buoyancy = total_gravity_force * buoyancy_ratio;
  float max_buoyancy_per_corner = max_total_buoyancy / 4.0f;

  // Get vehicle vertical position
  float vehicle_height = d->qpos[2];

  // Capture orientation quaternion
  for(int i = 0; i < 4; i++){
    _Qorn[i] = d->sensordata[i];
  }

  // Get site positions
  int site_ids[4];
  for(int i = 0; i < 4; i++) {
    char site_name[20];
    sprintf(site_name, "sfloat%d", i+1);
    site_ids[i] = mj_name2id(m, mjOBJ_SITE, site_name);
  }

  // Initialize total force on body to zero
  float total_buoyancy_force = 0.0f;
  float total_torque[3] = {0.0f, 0.0f, 0.0f};

  // Get body COM position
  float body_com[3];
  for(int i = 0; i < 3; i++) {
    body_com[i] = d->xpos[3 * body_id + i];
  }

  // Calculate buoyancy for each corner
  for(int i = 0; i < 4; i++) {
    if(site_ids[i] >= 0) {
      // Get the world position of this site
      float site_pos[3];
      for(int j = 0; j < 3; j++) {
        site_pos[j] = d->site_xpos[3 * site_ids[i] + j];
      }

      // Calculate submersion depth
      float submersion_depth = water_surface_height - site_pos[2];  // Z is vertical

      if(submersion_depth > 0.0f) {
        // Site is submerged
        float effective_depth = (submersion_depth > height) ? height : submersion_depth;
        float submerged_fraction = effective_depth / height;
        float buoyancy_force = max_buoyancy_per_corner * submerged_fraction;

        // Add to total vertical force
        total_buoyancy_force += buoyancy_force;

        // Calculate torque: r × F (force is purely vertical)
        float r[3];
        for(int j = 0; j < 3; j++) {
          r[j] = site_pos[j] - body_com[j];
        }

        // Cross product r × F where F = [0, 0, buoyancy_force]
        total_torque[0] += r[1] * buoyancy_force;   // rx * Fz
        total_torque[1] += -r[0] * buoyancy_force;  // -ry * Fz
        // torque[2] += 0 (no Z component since force is purely vertical)
      }
    }
  }

  // Apply the total force and torque to the body in world coordinates
  // xfrc_applied format: [fx, fy, fz, tx, ty, tz]
  d->xfrc_applied[6 * body_id + 0] += 0.0f;                // No X force
  d->xfrc_applied[6 * body_id + 1] += 0.0f;                // No Y force
  d->xfrc_applied[6 * body_id + 2] += total_buoyancy_force; // Z force (upward)
  d->xfrc_applied[6 * body_id + 3] += total_torque[0];     // X torque
  d->xfrc_applied[6 * body_id + 4] += total_torque[1];     // Y torque
  d->xfrc_applied[6 * body_id + 5] += total_torque[2];     // Z torque

  return true;
}

extern "C" bool roverPhysicsUpdatePlugNice(mjModel *m, mjData *d) {

  // Physical constants
  const float water_density = 1000.0f;  // kg/m³
  const float gravity = 9.806f;         // m/s²
  const float water_surface_height = 2.0f;  // Water surface level

  // Vehicle dimensions
  const float height = 0.254f;    // 10 inches - height of buoyancy volume

  // Get submarine mass and calculate gravitational force
  int body_id = mj_name2id(m, mjOBJ_BODY, "sub");
  float total_mass = (body_id >= 0) ? m->body_mass[body_id] : 9.0f;  // fallback to 9kg
  float total_gravity_force = total_mass * gravity;

  // Set buoyancy for slow sinking (adjust this ratio to control sink rate)
  const float buoyancy_ratio = 1.1f;  // 90% of gravity = slow sink
  float max_total_buoyancy = total_gravity_force * buoyancy_ratio;
  float max_buoyancy_per_corner = max_total_buoyancy / 4.0f;

  // Get vehicle vertical position (you mentioned qpos[2] is Y position)
  float vehicle_height = d->qpos[2];

  // Capture orientation quaternion
  for(int i = 0; i < 4; i++){
    _Qorn[i] = d->sensordata[i];
  }

  // Get site positions directly from MuJoCo data
  // Sites sfloat1-4 should correspond to site IDs 
  int site_ids[4];
  for(int i = 0; i < 4; i++) {
    char site_name[20];
    sprintf(site_name, "sfloat%d", i+1);
    site_ids[i] = mj_name2id(m, mjOBJ_SITE, site_name);
  }

  // Calculate buoyancy for each corner
  for(int i = 0; i < 4; i++) {
    float buoyancy_force = 0.0f;

    if(site_ids[i] >= 0) {
      // Get the Z-coordinate (vertical position) of this site
      float site_z = d->site_xpos[3 * site_ids[i] + 2];  // Z component

      // Calculate submersion depth
      float submersion_depth = water_surface_height - site_z;

      if(submersion_depth > 0.0f) {
        // Site is submerged
        // Clamp submersion to maximum effective depth
        float effective_depth = (submersion_depth > height) ? height : submersion_depth;

        // Buoyancy force proportional to submerged volume fraction
        float submerged_fraction = effective_depth / height;
        buoyancy_force = max_buoyancy_per_corner * submerged_fraction;
      }
    }

    // Apply purely vertical (upward) force
    d->ctrl[6 + i] = buoyancy_force;
  }

  return true;
}

extern "C" bool roverPhysicsUpdatePlug2(mjModel *m, mjData *d) {
  // printf("Physics Plug!\n");
  // printf("Updated Physics Plug!\n");
  float water_gain = 470.0f;
  float volume_displaced = 0.4572 *0.33782 *0.254;
  float max_bouyancy_force = 9.806f * volume_displaced * 1000;
  float start_height = 2;
  float force = (start_height - d->qpos[2]) * water_gain;

  // Capture Orientation
  //   [Px,Py,Pz, Ow,Ox,Oy,Oz]
  // Orientation: ^_________^
  for(int i=0; i<4; i++){
    _Qorn[i] = d->sensordata[i];
  }

  if(d->qpos[2] <= start_height - 0.1 ){
    float fnew = (force < max_bouyancy_force) ? (force) : (max_bouyancy_force);
    d->ctrl[6] = fnew/4.0;
    d->ctrl[7] = fnew/4.0;
    d->ctrl[8] = fnew/4.0;
    d->ctrl[9] = fnew/4.0;
    // printf("UP:%0.4f | MAX_B: %0.4f\n",fnew, max_bouyancy_force);
  }
  if(d->qpos[2] >= start_height + 0.1 ){
    d->ctrl[6] = ((1 - d->qpos[2]) * water_gain)/4.0;
    d->ctrl[7] = ((1 - d->qpos[2]) * water_gain)/4.0;
    d->ctrl[8] = ((1 - d->qpos[2]) * water_gain)/4.0;
    d->ctrl[9] = ((1 - d->qpos[2]) * water_gain)/4.0;
    // printf("Down:%0.4f | MAX_B: %0.4f\n",force, max_bouyancy_force);
  }

  quaternion_to_euler(_Qorn[1], _Qorn[2], _Qorn[3], _Qorn[0], &_orn[0], &_orn[1], &_orn[2]);

  // TODO: Create figure to show forces
  return true;
}
