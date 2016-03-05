
//***************************************************
//                   Define
//***************************************************
//flying mode
#define takeoff_mode        0
#define circle_mode         1
#define looping_mode        2
#define glide_mode          3
#define roll_mode           4
#define back_glide_mode     5
#define half_circle_mode    6
#define upclimb_mode        7
#define snake_mode          8

// Trajectory duration
#define takeoff_time            10e6
#define circle_time             20e6
#define rolling_time            10e6
#define looping_duration        8e6
#define half_circle_duration    17e6
#define climb_time              10e6

    //zeros time space zone
#define zero_space_size     0.5
#define center_zero_space_x -365
#define center_zero_space_y -400
#define center_zero_space_z -0.87
#define time_before_retesting_zero_time_condition 10e6

//***************************************************
//             Various Flying functions
//***************************************************
    // initialization when using trajectory function of time
    struct vector traj_initialize (struct vector &desired_path){

	
        desired_path.x = center_zero_space_x;//
        desired_path.y = center_zero_space_y;//
        desired_path.z = center_zero_space_z;//

        
        return(desired_path);
	}

    // taking off
	struct vector traj_takeoff (struct vector &desired_path, struct vector current_location){
	
	    struct vector delta_p; 
	    desired_path.x += 0.0 * static_cast<float>(PERIOD) /1e6;
        desired_path.y += 50.0 * static_cast<float>(PERIOD) /1e6;   // forward(y) direction incrementation for desired path in every second 
        desired_path.z -= 15.0 * static_cast<float>(PERIOD) /1e6;   // height incrementation, note minus value (NED)
      
        // calculate out the delta L, distance between desired location of the plane and the current location
        delta_p.x = desired_path.x - current_location.x;
        delta_p.y = desired_path.y - current_location.y;
        delta_p.z = desired_path.z- current_location.z;     
	
	return(delta_p);
	}
	
    // climb up mode
	struct vector traj_climbup (struct vector &desired_path, struct vector current_location){
	struct vector delta_p; 

    	desired_path.x += 0.0 * static_cast<float>(PERIOD) /1e6;
        desired_path.y += 50.0 * static_cast<float>(PERIOD) /1e6;
        desired_path.z -= 25.0 * static_cast<float>(PERIOD) /1e6;

        delta_p.x = desired_path.x - current_location.x;
        delta_p.y = desired_path.y - current_location.y;
        delta_p.z = desired_path.z- current_location.z;     

    return(delta_p);
	}

    // dive mode, not yet used
    struct vector traj_dive (struct vector &desired_path, struct vector current_location){
    struct vector delta_p; 

        desired_path.x += 0.0 * static_cast<float>(PERIOD) /1e6;
        desired_path.y += 50.0 * static_cast<float>(PERIOD) /1e6;
        desired_path.z += 20.0 * static_cast<float>(PERIOD) /1e6;

        delta_p.x = desired_path.x - current_location.x;
        delta_p.y = desired_path.y - current_location.y;
        delta_p.z = desired_path.z- current_location.z;     

    return(delta_p);
    }
	
	struct vector traj_glide (struct vector &desired_path, struct vector current_location){
	struct vector delta_p; 

        desired_path.x += 0.0 * static_cast<float>(PERIOD) /1e6; 
        desired_path.y += 50.0 * static_cast<float>(PERIOD) /1e6; // 50 for entering circle
        desired_path.z -= 0.0 * static_cast<float>(PERIOD) /1e6;// when circle = 0
        
        delta_p.x = desired_path.x - current_location.x;
        delta_p.y = desired_path.y - current_location.y;
        delta_p.z = desired_path.z- current_location.z;     
        
    return(delta_p);
	}

	struct vector traj_back_glide (struct vector &desired_path, struct vector current_location){
	struct vector delta_p; 

        desired_path.x += 0.0 * static_cast<float>(PERIOD) /1e6; 
        desired_path.y -= 50.0 * static_cast<float>(PERIOD) /1e6; // 50 for entering circle
        desired_path.z -= 0.0 * static_cast<float>(PERIOD) /1e6;// when circle = 0
        
        delta_p.x = desired_path.x - current_location.x;
        delta_p.y = desired_path.y - current_location.y;
        delta_p.z = desired_path.z- current_location.z;     
        
    return(delta_p);
	}
	
	struct vector traj_loop (struct vector &desired_path, struct vector current_location, uint32_t time, uint32_t timer){
    	struct vector delta_p; 
        uint32_t looping_time = 10e6; // need reconsider, faster maybe better for looping

    	desired_path.x +=  0.0;
    	desired_path.y -=  (100.0*cos(360.0*(M_PI/180.0)*((time+1e6-timer)/(looping_time))))*static_cast<float>(PERIOD)/1e6;   // here radius is 100, need to reconsider
    	desired_path.z +=  (100.0*sin(360.0*(M_PI/180.0)*(time+1e6-timer)/(looping_time)))*static_cast<float>(PERIOD)/1e6;     //

    
        delta_p.x = desired_path.x - current_location.x;
        delta_p.y = desired_path.y - current_location.y;
        delta_p.z = desired_path.z- current_location.z;         

    return(delta_p);
	}
	
    struct vector traj_circle (struct vector &desired_path, struct vector current_location, uint32_t time){
    struct vector delta_p; 

        desired_path.x += (70.0*cos(360.0*(M_PI/180.0)*(time)/(30e6)))*static_cast<float>(PERIOD)/1e6; //
        desired_path.y += (70.0*sin(360.0*(M_PI/180.0)*(time)/(30e6)))*static_cast<float>(PERIOD)/1e6; //
        desired_path.z -= 1.0 * static_cast<float>(PERIOD) /1e6;
        
        delta_p.x = desired_path.x - current_location.x;
        delta_p.y = desired_path.y - current_location.y;
        delta_p.z = desired_path.z- current_location.z;     
        
    return(delta_p);
    }

    struct vector traj_snake (struct vector &desired_path, struct vector current_location, uint32_t time){
    struct vector delta_p; 

        //use derivative, added up to integral * static_cast<float>(PERIOD) /1e6; around 80-100 degrees
        desired_path.x += 50.0*cos(360.0*(M_PI/180.0)*(time-20e6)/(20e6))*static_cast<float>(PERIOD)/1e6;  
        desired_path.y += 50.0 *static_cast<float>(PERIOD)/1e6; //
        desired_path.z -= 1.0* static_cast<float>(PERIOD) /1e6;    // exchange with function in x, to achieve an up/down snake mode
        
        delta_p.x = desired_path.x - current_location.x;
        delta_p.y = desired_path.y - current_location.y;
        delta_p.z = desired_path.z- current_location.z;     
        
    return(delta_p);
    }

// actually real roll actions in autopilot, 
struct vector traj_roll (struct vector &desired_path, struct vector current_location){
    struct vector delta_p; 

        desired_path.x += 0.0 * static_cast<float>(PERIOD) /1e6; 
        desired_path.y += 50.0 * static_cast<float>(PERIOD) /1e6; // 50 for entering circle
        desired_path.z -= 0.0 * static_cast<float>(PERIOD) /1e6;// when circle = 0
        
        delta_p.x = desired_path.x - current_location.x;
        delta_p.y = desired_path.y - current_location.y;
        delta_p.z = desired_path.z- current_location.z;     
        
    return(delta_p);
    }
   

//***********************************************************
//         Trajectory function based on API commands
//***********************************************************
// Restore initial conditions of the path, when a reset has taken place.
struct vector choose_traj (uint8_t firstLoop, int Plane_flying_current_state, int Plane_flying_next_state, BOOL plane_flying_busy, struct vector &desired_path, struct vector current_location, uint32_t relative_time, uint32_t &timer, float &phiRef){
        struct vector trajectory_refgnd;

        if(firstLoop){
            desired_path.x = center_zero_space_x;
            desired_path.y = center_zero_space_y;
            desired_path.z = center_zero_space_z;
            phiRef = 0;
        }
        // If the plane is not busy, just jump into next flying mode/state
        if(plane_flying_busy==FALSE){
         Plane_flying_current_state=Plane_flying_next_state;    
         timer = relative_time;     // Set timer to count time in a certain flying mode
         }
        
        // Switch different flying mode according to the API typed-in commands, such as 'ft' for takeoff. Add or change in API_perso.cpp, starting from line 111
        switch(Plane_flying_current_state)
        {
            case takeoff_mode:
                plane_flying_busy=TRUE;
                trajectory_refgnd = traj_takeoff(desired_path, current_location); // function defined in Trajectory.cpp
                if(relative_time-timer > takeoff_time){
                    plane_flying_busy=FALSE;    // Ready to take next state after some time
                    timer = relative_time;      // Refresh timer for the next trajectory change 
                    if(Plane_flying_next_state==Plane_flying_current_state) Plane_flying_next_state=glide_mode; //security, avoid false re-typing
                    }
                break;
                
            case circle_mode:
                trajectory_refgnd = traj_circle(desired_path, current_location, relative_time-timer);
                if(relative_time-timer > circle_time){
                    plane_flying_busy=FALSE;
                    timer = relative_time;
                    }
                break; 
                
            case half_circle_mode:  // Turn half a circle, better to perform loop acrobatics
                plane_flying_busy=TRUE;
                trajectory_refgnd = traj_circle(desired_path, current_location, relative_time-timer);
                if(relative_time-timer > half_circle_duration){
                    plane_flying_busy=FALSE;
                    timer = relative_time;
                    if(Plane_flying_next_state==Plane_flying_current_state) Plane_flying_next_state=back_glide_mode; //security
                    }
                break; 


            // Barrel looping needs high throttle and pitch acceleration, during test, desired path in looping might has not enough acceleration.
            // Barrel looping achieved by first glide forward and then glide backward, plane will adjust itself in a looping style.
            // looping function works when trying first roll upside down and perform a downward looping
            case looping_mode:
                plane_flying_busy=TRUE;
                trajectory_refgnd = traj_glide(desired_path, current_location);
                if(relative_time-timer > looping_duration){
                    trajectory_refgnd = traj_back_glide(desired_path, current_location);
                    plane_flying_busy=FALSE;
                    timer = relative_time;
                    if(Plane_flying_next_state==Plane_flying_current_state) Plane_flying_next_state=glide_mode; //security
                    }
                break; 

            /*  // 
                plane_flying_busy=TRUE;
                trajectory_refgnd = traj_loop(desired_path, current_location, relative_time, timer);
                phiRef += 180.0/5e6*static_cast<float>(PERIOD);
                if (360>phiRef>180){
                    phiRef=180;
                }
                if(relative_time-timer > looping_duration){ // need reconsider, in accordance with the looping func
                    plane_flying_busy=FALSE;
                    timer = relative_time;
                    if(Plane_flying_next_state==Plane_flying_current_state) Plane_flying_next_state=glide_mode; //security
                    }

                break;
            */
                
            case glide_mode:    // fly forward, these three modes are maintained without disturbance
                trajectory_refgnd = traj_glide(desired_path, current_location);
                plane_flying_busy=FALSE;
                timer = relative_time;
                break; 
                
            case back_glide_mode: // fly backward
                trajectory_refgnd = traj_back_glide(desired_path, current_location);
                plane_flying_busy=FALSE;
                timer = relative_time;
                break;
                
            case snake_mode: // snake mode, fly sinusoidally left/right or up/down
                trajectory_refgnd = traj_snake(desired_path, current_location, relative_time);
                plane_flying_busy=FALSE;
                timer = relative_time;
                break;
                                
            case roll_mode: // can be shrinked 
                plane_flying_busy=TRUE;
                trajectory_refgnd = traj_roll(desired_path, current_location);
                phiRef += 180.0/5e6*static_cast<float>(PERIOD);
                if (phiRef>180){
                    phiRef=180;
                }else{
                    phiRef += 180.0/5e6*static_cast<float>(PERIOD);
                    if (phiRef>360){
                        phiRef=0;
                    }
                }
                if(relative_time-timer > rolling_time){
                    plane_flying_busy=FALSE;
                    timer = relative_time;
                    if(Plane_flying_next_state==Plane_flying_current_state) Plane_flying_next_state=glide_mode; //security
                    }
 
                break;
                
            case upclimb_mode: // higher climb rate after takeoff
                plane_flying_busy=TRUE;
                trajectory_refgnd = traj_climbup(desired_path, current_location);
                if(relative_time-timer > climb_time){
                    plane_flying_busy=FALSE;
                    timer = relative_time;
                    if(Plane_flying_next_state==Plane_flying_current_state) Plane_flying_next_state=glide_mode; //security
                    }
                break;                
                
            //if there are a problem with the value
            default:
                Plane_flying_current_state=glide_mode;
                Plane_flying_next_state=glide_mode;
                break;
        }

        return(trajectory_refgnd);
    }

/*
// This trajectory function is based on time, free from typing in commands, you can adjust periods for different acrobatics
struct vector FlyTrajectory (uint8_t firstLoop, struct vector &desired_path, struct vector current_location, uint32_t time, uint32_t tstart, uint32_t tend, float phiRef){
        struct vector trajectory_refgnd;
         if (firstLoop){
            desired_path = traj_initialize(desired_path);

            phiRef = 0;
                //firstLoop = 0; Switched off down at printing!
            }
        if (time<3e6 && (-366<current_location.x<-364)&&(-401<current_location.y<-399)&&(-1.87<current_location.z<0.27)){

                desired_path = traj_initialize(desired_path); // when restarted, reinitialize plane position

            phiRef = 0;
            }
          if (time<10e6){
            
                trajectory_refgnd = traj_takeoff(desired_path, current_location);
                }else if (time<20e6){
                    
                trajectory_refgnd = traj_climbup(desired_path, current_location);
                //desired_path = traj_initialize(current_location);
                }
                
            else if (time<30e6){
                //desired_path = traj_initialize(current_location);
                //trajectory_refgnd = traj_loop(desired_path, current_location, time, tstart, tend);
                //trajectory_refgnd = traj_glide(desired_path, current_location);
                trajectory_refgnd = traj_roll(desired_path, current_location);
                phiRef += 180.0/5e6*static_cast<float>(PERIOD);
                if (360>phiRef>180 ){
                phiRef=180;
                }
                if (phiRef>360){
                phiRef=0;
                }           
            }
   
            else {   
                //trajectory_refgnd = traj_climbup(desired_path, current_location);
                //trajectory_refgnd = traj_circle(desired_path, current_location, time);
                //trajectory_refgnd = traj_snake(desired_path, current_location, time);
                //trajectory_refgnd = traj_loop(desired_path, current_location, time, tstart, tend);
                //trajectory_refgnd = traj_roll(desired_path, current_location, phiRef);
                trajectory_refgnd = traj_glide(desired_path, current_location);
            }
          
            return (trajectory_refgnd);

}
*/
