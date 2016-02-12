	struct vector traj_initialize (struct vector current_location){
	struct vector allola;
	
        allola.x = -365.0;//current_location.x;
        allola.y = -400.0;//current_location.y;
        allola.z = -0.87;//current_location.z;
        
        return(allola);
	}
	
    struct vector traj_refresh (struct vector current_location){
    struct vector allola;
    float rad = 50.0;
        allola.x = current_location.x;
        allola.y = current_location.y;
        allola.z = current_location.z;
        
        return(allola);
    }

	struct vector traj_takeoff (struct vector &desired_path, struct vector current_location){
	
	    struct vector delta_p; 
	    desired_path.x += 0.0 * static_cast<float>(PERIOD) /1e6;
        desired_path.y += 50.0 * static_cast<float>(PERIOD) /1e6;
        desired_path.z -= 5.0 * static_cast<float>(PERIOD) /1e6; 
      
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
	
	struct vector traj_cruise (struct vector &desired_path, struct vector current_location){
	struct vector delta_p; 

        desired_path.x += 0.0 * static_cast<float>(PERIOD) /1e6; 
        desired_path.y += 50.0 * static_cast<float>(PERIOD) /1e6; // 50 for entering circle
        desired_path.z -= 0.0 * static_cast<float>(PERIOD) /1e6;// when circle = 0
        
        delta_p.x = desired_path.x - current_location.x;
        delta_p.y = desired_path.y - current_location.y;
        delta_p.z = desired_path.z- current_location.z;     
        
    return(delta_p);
	}
	
	struct vector traj_loop (struct vector &desired_path, struct vector current_location, uint32_t time, uint32_t tstart, uint32_t tend){
    	struct vector delta_p; 

    	desired_path.x +=  0.0;
    	desired_path.y +=  (40.0*cos(360.0*(M_PI/180.0)*((time+1e6-tstart)/(tend-tstart)))+40)*static_cast<float>(PERIOD)/1e6;//
    	desired_path.z -=  (40.0*sin(360.0*(M_PI/180.0)*(time+1e6-tstart)/(tend-tstart))+10)*static_cast<float>(PERIOD)/1e6;//

    
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

struct vector FlyTrajectory (uint8_t firstLoop, struct vector &desired_path, struct vector current_location, uint32_t time, uint32_t tstart, uint32_t tend){
        struct vector trajectory_refgnd;
	     if (firstLoop){
                desired_path = traj_initialize(current_location);

                //firstLoop = 0; Switched off down at printing!
            }
            if (time<3e6 && (-366<current_location.x<-364)&&(-401<current_location.y<-399)&&(-1.87<current_location.z<0.27)){

                desired_path = traj_initialize(current_location); // when restarted, reinitialize plane position

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
                trajectory_refgnd = traj_cruise(desired_path, current_location);
            }
   
            else {   
                //trajectory_refgnd = traj_climbup(desired_path, current_location);
                trajectory_refgnd = traj_circle(desired_path, current_location, time);
                //trajectory_refgnd = traj_snake(desired_path, current_location, time);
                //trajectory_refgnd = traj_loop(desired_path, current_location, time, tstart, tend);
            }
          
            return (trajectory_refgnd);

}

