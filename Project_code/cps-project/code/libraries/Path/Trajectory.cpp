    struct vector traj_initialize (struct vector &desired_path){

	
        desired_path.x = center_zero_space_x;//
        desired_path.y = center_zero_space_y;//
        desired_path.z = center_zero_space_z;//

        
        return(desired_path);
	}

	struct vector traj_takeoff (struct vector &desired_path, struct vector current_location){
	
	    struct vector delta_p; 
	    desired_path.x += 0.0 * static_cast<float>(PERIOD) /1e6;
        desired_path.y += 40.0 * static_cast<float>(PERIOD) /1e6;
        desired_path.z -= 15.0 * static_cast<float>(PERIOD) /1e6; 
      
        delta_p.x = desired_path.x - current_location.x;
        delta_p.y = desired_path.y - current_location.y;
        delta_p.z = desired_path.z- current_location.z;     
	
	return(delta_p);
	}
	
	struct vector traj_climbup (struct vector &desired_path, struct vector current_location){
	struct vector delta_p; 

    	desired_path.x += 0.0 * static_cast<float>(PERIOD) /1e6;
        desired_path.y += 50.0 * static_cast<float>(PERIOD) /1e6;
        desired_path.z -= 20.0 * static_cast<float>(PERIOD) /1e6;

        delta_p.x = desired_path.x - current_location.x;
        delta_p.y = desired_path.y - current_location.y;
        delta_p.z = desired_path.z- current_location.z;     

    return(delta_p);
	}

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
        uint32_t looping_time = 10e6; // need reconsider

    	desired_path.x +=  0.0;
    	desired_path.y -=  (100.0*cos(360.0*(M_PI/180.0)*((time+1e6-timer)/(looping_time))))*static_cast<float>(PERIOD)/1e6;//
    	desired_path.z +=  (100.0*sin(360.0*(M_PI/180.0)*(time+1e6-timer)/(looping_time)))*static_cast<float>(PERIOD)/1e6;//

    
        delta_p.x = desired_path.x - current_location.x;
        delta_p.y = desired_path.y - current_location.y;
        delta_p.z = desired_path.z- current_location.z;         

    return(delta_p);
	}
	
    struct vector traj_circle (struct vector &desired_path, struct vector current_location, uint32_t time){
    struct vector delta_p; 

        desired_path.x += (70.0*cos(360.0*(M_PI/180.0)*(time-30e6)/(30e6)))*static_cast<float>(PERIOD)/1e6; //-20e6)/(
        desired_path.y += (70.0*sin(360.0*(M_PI/180.0)*(time-30e6)/(30e6)))*static_cast<float>(PERIOD)/1e6; //
        desired_path.z -= 1.0 * static_cast<float>(PERIOD) /1e6;
        
        delta_p.x = desired_path.x - current_location.x;
        delta_p.y = desired_path.y - current_location.y;
        delta_p.z = desired_path.z- current_location.z;     
        
    return(delta_p);
    }

    struct vector traj_snake (struct vector &desired_path, struct vector current_location, uint32_t time){
    struct vector delta_p; 

        desired_path.x += 50.0*cos(360.0*(M_PI/180.0)*(time-20e6)/(20e6))*static_cast<float>(PERIOD)/1e6;  //use derivative, added up to integral * static_cast<float>(PERIOD) /1e6; around 80-100 degrees
        desired_path.y += 50.0 *static_cast<float>(PERIOD)/1e6; //10.0 * static_cast<float>(PERIOD) /1e6; 
        desired_path.z -= 1.0* static_cast<float>(PERIOD) /1e6;;//(50.0*sin(360.0*(M_PI/180.0)*(time-20e6)/(20e6)))*static_cast<float>(PERIOD)/1e6; //
        
        delta_p.x = desired_path.x - current_location.x;
        delta_p.y = desired_path.y - current_location.y;
        delta_p.z = desired_path.z- current_location.z;     
        
    return(delta_p);
    }

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
   /*    
void traj_sel(const enum trajName trajnames){
    int i;
    switch(trajnames) {
        case takeoff:
        {

        }


    


}
     */      

/*
struct vector FlyTrajectory (uint8_t firstLoop, struct vector &desired_path, struct vector current_location, uint32_t time, uint32_t tstart, uint32_t tend, float phiRef, bool testLock, bool testLock2){
        struct vector trajectory_refgnd;
         if (firstLoop){
            desired_path = traj_initialize(desired_path);

            phiRef = 0;
            testLock = FALSE;
            testLock2 = FALSE;
                //firstLoop = 0; Switched off down at printing!
            }
        if (time<3e6 && (-366<current_location.x<-364)&&(-401<current_location.y<-399)&&(-1.87<current_location.z<0.27)){

                desired_path = traj_initialize(desired_path); // when restarted, reinitialize plane position

            phiRef = 0;
            testLock = FALSE;
            testLock2 = FALSE;
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
                if (360>phiRef>180 || testLock==TRUE){
                phiRef=180;
                testLock=TRUE;
                }
                if (phiRef>360 || testLock2==TRUE){
                phiRef=0;
                testLock2=TRUE;
                }           
            }
   
            else {   
                //trajectory_refgnd = traj_climbup(desired_path, current_location);
                //trajectory_refgnd = traj_circle(desired_path, current_location, time);
                //trajectory_refgnd = traj_snake(desired_path, current_location, time);
                //trajectory_refgnd = traj_loop(desired_path, current_location, time, tstart, tend);
                //trajectory_refgnd = traj_roll(desired_path, current_location, phiRef, testLock);
                trajectory_refgnd = traj_glide(desired_path, current_location);
            }
          
            return (trajectory_refgnd);

}
*/
