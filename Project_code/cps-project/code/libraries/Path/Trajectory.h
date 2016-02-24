#ifndef __TRAJECTORY_H__
#define __TRAJECTORY_H__

#include "Trajectory.cpp"

enum trajName {takeoff, climbup, snake, circle, glide, rolling, looping};
/*
enum class trajName: char {
    takeoff = 't', 
    climbup = 'u', 
    snake = 's', 
    circle = 'c', 
    glide = 'g', 
    rolling = 'r', 
    looping = 'l',
};
*/
    struct vector traj_initialize (struct vector &desired_path);
    struct vector traj_takeoff (struct vector &desired_path, struct vector current_location);
    struct vector traj_climbup (struct vector &desired_path, struct vector current_location);
    struct vector traj_glide (struct vector &desired_path, struct vector current_location);
    struct vector traj_loop (struct vector &desired_path, struct vector current_location, uint32_t timer);
    struct vector traj_snake (struct vector &desired_path, struct vector current_location, uint32_t time);
    struct vector traj_dive (struct vector &desired_path, struct vector current_location);
    struct vector traj_roll (struct vector &desired_path, struct vector current_location);

    struct vector FlyTrajectory (uint8_t firstLoop, struct vector &desired_path, struct vector current_location, uint32_t time, uint32_t timer, float phiRef, bool testLock, bool testLock2);

uint32_t looping_time = 10e6
#endif
