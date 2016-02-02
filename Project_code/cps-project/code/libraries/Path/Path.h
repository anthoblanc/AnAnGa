#ifndef __PATH_H__
#define __PATH_H__


// problem: desired y - current y = negative

struct vector traj_initialize (struct vector current_location) { // initialize positions of the plane
	struct vector desired_path;
	
        desired_path.x = current_location.x + 0.0;
        desired_path.y = current_location.y + 100.0;
        desired_path.z = current_location.z - 70.0; // strangely set the desired height, pathned here changed in autopilot didn't change 
        
        return (desired_path);
}
// take off
struct vector traj_takeoff (struct vector desired_path, struct vector current_location, struct vector inertial_velocity){
	
	struct vector delta_p; 
	/*
	desired_path.x += 0.0 * inertial_velocity.x * static_cast<float>(PERIOD) /1e6;
        desired_path.y += 100.0 * inertial_velocity.y * static_cast<float>(PERIOD) /1e6;
        desired_path.z -= 30.0 * inertial_velocity.z * static_cast<float>(PERIOD) /1e6;
      */
        delta_p.x = 0.0;//desired_path.x - current_location.x;
        delta_p.y = desired_path.y - current_location.y;
        delta_p.z = desired_path.z- current_location.z;     
	
	return (delta_p);
}

struct vector traj_climbup (struct vector desired_path, struct vector current_location, struct vector inertial_velocity) {
	struct vector delta_p; 
/*
	desired_path.x += 0.0 * static_cast<float>(PERIOD) /1e6;//* inertial_velocity.x * static_cast<float>(PERIOD) /1e6;
        desired_path.y += 100.0 * static_cast<float>(PERIOD) /1e6;//* inertial_velocity.y * static_cast<float>(PERIOD) /1e6;
        desired_path.z -= 50.0 * static_cast<float>(PERIOD) /1e6;//* inertial_velocity.z * static_cast<float>(PERIOD) /1e6;
*/
        delta_p.x = 0.0;//desired_path.x - current_location.x;
        delta_p.y = desired_path.y - current_location.y;
        delta_p.z = desired_path.z- current_location.z;     
            
	return (delta_p);
}

struct vector traj_cruise (struct vector desired_path, struct vector current_location, struct vector inertial_velocity){
	struct vector delta_p; 
	desired_path.x += 0.0 * static_cast<float>(PERIOD) /1e6; 
        desired_path.y += 10.0 * static_cast<float>(PERIOD) /1e6; 
        desired_path.z -= 0.0 * static_cast<float>(PERIOD) /1e6;
        
        delta_p.x = 0.0; //desired_path.x - current_location.x;
        delta_p.y = 100.0; //desired_path.y - current_location.y;
        delta_p.z = 0.0; //desired_path.z- current_location.z;     
        
        return (delta_p);
}

struct vector traj_loop (struct vector desired_path, struct vector current_location, uint32_t time, uint32_t tstart, uint32_t tend){
    	struct vector delta_p; 
	struct vector pnPePdtmp0;
	struct vector pnPePdtmp;
 	if (time >= tstart && time <= (tstart + 1000)){     //coordinate around loop start
            pnPePdtmp0 = current_location;
        }
        if(time >= (tstart + 1000) && time <= (tstart + 2000)){ // coordinate a bit after
            pnPePdtmp = current_location;
        }


    desired_path.x +=  2e3*(pnPePdtmp.x - pnPePdtmp0.x)*sin(360.0*(M_PI/180.0)*(time+2e6-tstart)/(tend-tstart));
    desired_path.y +=  2e3*(pnPePdtmp.y - pnPePdtmp0.y)*sin(360.0*(M_PI/180.0)*(time+2e6-tstart)/(tend-tstart));
    desired_path.z -=  200.0*sin((360.0*(M_PI/180.0)*(time+2e6-tstart)/(tend-tstart)));//sqrt(1-cos(360.0*(M_PI/180.0)*(time+2e6-tstart)/(tend-tstart))); // radius = 200
    // give altitude coordinate after 2 sec
    
        delta_p.x = 0.0;//desired_path.x - current_location.x;
        delta_p.y = desired_path.y - current_location.y;
        delta_p.z = desired_path.z - current_location.z;         
    return(delta_p);
    }


struct vector pathned;


#endif

			

            /*	// Path Control
                // use two points before loop to decide direction
                if (time <tstart){     //
                trajectory_refgnd.x = 6;
                trajectory_refgnd.y = 80;
                 trajectory_refgnd.z = -10;
                }
                if (time >= (tstart - 1500000) && time <= (tstart - 1000000) ){     //coordinate 1 sec before loop
                pnPePdtmp0 = stateVars.pnPePd;
                }
                 if(time >= (tstart - 5000000) && time <= tstart) { // coordinate just before loop
                 pnPePdtmp = stateVars.pnPePd;
                 }
                // 20s -> 40s, perform loop, give target coordinates two secs in the future
                if(time >= tstart && time <= tend)
                {
                    pathned = pathloop ( pnPePdtmp0, pnPePdtmp, time, tstart, tend );

                    //calculate the deisred path "L"
                    trajectory_refgnd.x = pathned.x - stateVars.pnPePd.x;
                    trajectory_refgnd.y = pathned.y - stateVars.pnPePd.y;
                    trajectory_refgnd.z = pathned.z- stateVars.pnPePd.z;
                }
            */

  

            // Calculating differential Trajectory
/*
            trajectory_refgnd.x = pathned.x - stateVars.pnPePd.x;
            trajectory_refgnd.y = pathned.y - stateVars.pnPePd.y;
            trajectory_refgnd.z = pathned.z - stateVars.pnPePd.z;
*/

