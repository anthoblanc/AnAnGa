#ifndef __TRAJECTORY_H__
#define __TRAJECTORY_H__


#include "Trajectory.cpp"
/*
class Trajectory {
	public: 
	
	Trajectory(struct vector desired_path, struct vector current_location, uint32_t time, uint32_t tstart, uint32_t tend);
	
};
*/
    
    struct vector traj_initialize (struct vector current_location);
    struct vector traj_takeoff (struct vector desired_path, struct vector current_location);
    struct vector traj_climbup (struct vector desired_path, struct vector current_location);
    struct vector traj_cruise (struct vector desired_path, struct vector current_location);
    struct vector traj_loop (struct vector desired_path, struct vector current_location, uint32_t time, uint32_t tstart, uint32_t tend);

    struct vector FlyTrajectory (uint8_t firstLoop, struct vector desired_path, struct vector current_location, uint32_t time, uint32_t tstart, uint32_t tend);

#endif
