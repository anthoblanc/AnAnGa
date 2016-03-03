#ifndef __TRAJECTORY_H__
#define __TRAJECTORY_H__

#include "Trajectory.cpp"

    struct vector traj_initialize (struct vector &desired_path);
    struct vector traj_takeoff (struct vector &desired_path, struct vector current_location);
    struct vector traj_climbup (struct vector &desired_path, struct vector current_location);
    struct vector traj_glide (struct vector &desired_path, struct vector current_location);
    struct vector traj_back_glide (struct vector &desired_path, struct vector current_location);
    struct vector traj_loop (struct vector &desired_path, struct vector current_location, uint32_t timer);
    struct vector traj_snake (struct vector &desired_path, struct vector current_location, uint32_t time);
    struct vector traj_dive (struct vector &desired_path, struct vector current_location);  // not yet used in Autopilot, add when necessary
    struct vector traj_roll (struct vector &desired_path, struct vector current_location);

    struct vector FlyTrajectory (uint8_t firstLoop, struct vector &desired_path, struct vector current_location, uint32_t time, uint32_t timer, float phiRef);
    struct vector choose_traj (uint8_t firstLoop, int Plane_flying_current_state, int Plane_flying_next_state, BOOL plane_flying_busy, struct vector &desired_path, struct vector current_location, uint32_t time, uint32_t &timer, float &phiRef);

#endif
