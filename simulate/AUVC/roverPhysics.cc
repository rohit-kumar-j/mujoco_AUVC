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

extern "C" bool roverPhysicsUpdatePlug(mjModel *m, mjData *d) {
    
    // Physical constants
    const float water_density = 1000.0f;  // kg/m³
    const float gravity = 9.806f;         // m/s²
    const float water_surface_height = 1.0f;  // Water surface level
    
    // Vehicle dimensions
    const float height = 0.254f;    // 10 inches - height of buoyancy volume
    
    // Calculate volume per corner (from your original calculation)
    const float volume_displaced = 0.4572f * 0.33782f * 0.254f;
    const float volume_per_corner = volume_displaced / 4.0f;
    
    // Maximum buoyancy force per corner
    const float max_buoyancy_per_corner = gravity * volume_per_corner * water_density;
    
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
