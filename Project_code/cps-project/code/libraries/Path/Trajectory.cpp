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

/*	desired_path.x += 0.0 * static_cast<float>(PERIOD) /1e6; 
        desired_path.y += 10.0 * static_cast<float>(PERIOD) /1e6; 
        desired_path.z -= 0.0 * static_cast<float>(PERIOD) /1e6;
 */       
        delta_p.x = 0.0; //desired_path.x - current_location.x;
        delta_p.y = 100.0; //desired_path.y - current_location.y;
        delta_p.z = -5.0; //desired_path.z- current_location.z;     
        
    return(delta_p);
	}
	
	struct vector traj_loop (struct vector &desired_path, struct vector current_location, uint32_t time, uint32_t tstart, uint32_t tend){
    	struct vector delta_p; 
	/* struct vector pnPePdtmp0;
	struct vector pnPePdtmp;
 	if (time >= tstart && time <= (tstart + 1000)){     //coordinate around loop start
            pnPePdtmp0 = current_location;
        }
        if(time >= (tstart + 1000) && time <= (tstart + 2000)){ // coordinate a bit after
            pnPePdtmp = current_location;
        }
        else if (time > (tstart + 2000)){ */
    	desired_path.x +=  0.0;//2e3*(pnPePdtmp0.x - pnPePdtmp.x)*sin(360.0*(M_PI/180.0)*(time+1e6-tstart)/(tend-tstart));
    	desired_path.y +=  300.0*cos(360.0*(M_PI/180.0)*((time+3e6-tstart)/(tend-tstart)))*static_cast<float>(PERIOD)/1e6;//2e3*(pnPePdtmp0.y - pnPePdtmp.y)*sin(360.0*(M_PI/180.0)*(time+1e6-tstart)/(tend-tstart));
    	desired_path.z -=  300.0*sin(360.0*(M_PI/180.0)*(time+3e6-tstart)/(tend-tstart))*static_cast<float>(PERIOD)/1e6;//200.0*sin((360.0*(M_PI/180.0)*(time+1e6-tstart)/(tend-tstart)));
    	//sqrt(1-cos(360.0*(M_PI/180.0)*(time+2e6-tstart)/(tend-tstart))); // radius = 200
    	// give altitude coordinate after 2 sec
    
        delta_p.x = 0.0;//desired_path.x - current_location.x;
        delta_p.y = desired_path.y - current_location.y;
        delta_p.z = desired_path.z- current_location.z;         

    return(delta_p);
	}
	
    struct vector traj_circle (struct vector &desired_path, struct vector current_location, uint32_t time){
    struct vector delta_p; 

        desired_path.x += (100.0*cos(360.0*(M_PI/180.0)*(time-20e6)/(20e6)))*static_cast<float>(PERIOD)/1e6; //-20e6)/(
        desired_path.y += (100.0*sin(360.0*(M_PI/180.0)*(time-20e6)/(20e6)) + 40)*static_cast<float>(PERIOD)/1e6; //
        desired_path.z -= 2.0 * static_cast<float>(PERIOD) /1e6;
        
        delta_p.x = desired_path.x - current_location.x;
        delta_p.y = desired_path.y - current_location.y;
        delta_p.z = desired_path.z- current_location.z;     
        
    return(delta_p);
    }

    struct vector traj_snake (struct vector &desired_path, struct vector current_location, uint32_t time){
    struct vector delta_p; 

        desired_path.x += 50.0*cos(360.0*(M_PI/180.0)*(time-20e6)/(20e6))*static_cast<float>(PERIOD)/1e6;  //use derivative, added up to integral * static_cast<float>(PERIOD) /1e6; around 80-100 degrees
        desired_path.y += 40.0 *static_cast<float>(PERIOD)/1e6; //10.0 * static_cast<float>(PERIOD) /1e6; 
        desired_path.z -= 2.0* static_cast<float>(PERIOD) /1e6;;//(50.0*sin(360.0*(M_PI/180.0)*(time-20e6)/(20e6)))*static_cast<float>(PERIOD)/1e6; //
        
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
                }else if (time<30e6){
                    
                trajectory_refgnd = traj_climbup(desired_path, current_location);
                //desired_path = traj_initialize(current_location);
                }
                /*
            else if (time<30e6){
                //desired_path = traj_initialize(current_location);
                trajectory_refgnd = traj_loop(desired_path, current_location, time, tstart, tend);
            }
            */
          /*  else if(time<35e6){
                trajectory_refgnd = traj_snake(desired_path, current_location, time);
            }
           
            else if (time<20e6+1000){
                desired_path = traj_refresh(current_location);
            }
            */
           else if (time<31e6){
                //desired_path = traj_initialize(current_location);
                //trajectory_refgnd = traj_dive(desired_path, current_location);
                //trajectory_refgnd = traj_circle(desired_path, current_location, time);
            }
          
            else {   
                //trajectory_refgnd = traj_climbup(desired_path, current_location);
                //trajectory_refgnd = traj_climbup(desired_path, current_location);
                trajectory_refgnd = traj_snake(desired_path, current_location, time);
            }
          
            return (trajectory_refgnd);

}


/*
  struct vector trajectory_refgnd;
         if (firstLoop){
                desired_path = traj_initialize(current_location);

                //firstLoop = 0; Switched off down at printing!
            }
          if (time<10e6){
           
            desired_path.x += 0.0 *  static_cast<float>(PERIOD) /1e6;
            desired_path.y += 80.0 * static_cast<float>(PERIOD) /1e6;
            desired_path.z -= 10.0 * static_cast<float>(PERIOD) /1e6;
      
                trajectory_refgnd = traj_takeoff(desired_path, current_location);
                }else if (time<20e6){
                    
                desired_path.x += 0.0 * static_cast<float>(PERIOD) /1e6;//* inertial_velocity.x * static_cast<float>(PERIOD) /1e6;
                desired_path.y += 100.0 * static_cast<float>(PERIOD) /1e6;//* inertial_velocity.y * static_cast<float>(PERIOD) /1e6;
                desired_path.z -= 20.0 * static_cast<float>(PERIOD) /1e6;//* inertial_velocity.z * static_cast<float>(PERIOD) /1e6;

                trajectory_refgnd = traj_climbup(desired_path, current_location);
                }
                
            else if (time<30e6){

                trajectory_refgnd = traj_loop(desired_path, current_location, time, tstart, tend);
            }
            else if(time<35e6){
                trajectory_refgnd = traj_climbup(desired_path, current_location);
            }
            else if (time<45e6){
                trajectory_refgnd = traj_circle(desired_path, current_location, time);
            }
            else {
                trajectory_refgnd = traj_cruise(desired_path, current_location);
            }
            return (trajectory_refgnd);
*/
