/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil
#define THISFIRMWARE "CPS-Autopilot-Project"

//***************************************************
//                   Define
//***************************************************


//General
typedef int           BOOL;
#define FALSE         0
#define TRUE          1
//Note: Boolean also exist in a library but we don't really need to do complex feature

#define NOT           !
#define AND           &&
#define OR            ||

//zeros time space zone
#define zero_space_size 0.5
#define center_zero_space_x -365
#define center_zero_space_y -400
#define center_zero_space_z -0.87
#define time_before_retesting_zero_time_condition 10e6

// Look Ahead Distance (Desired Length) [feet]
#define LookAheadDistance 40

//interface
#define size_buffer_interface 20

//flying mode
#define takeoff_mode        0
#define circle_mode         1
#define looping_mode        2
#define glide_mode          3
#define roll_mode           4

// Trajectory time
#define takeoff_time 10e6
#define circle_time 20e6
#define rolling_time 15e6
#define looping_duration 10e6


//***************************************************
// Libraries
//***************************************************

// All standard configurations
#include <AP_Common.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <Wire.h>
#include <RC_Channel.h>
#include <AP_Motors.h>
#include <AP_Notify.h>
#include <AP_Curve.h>

// Export of all declarations, initially given.
#include <StandardConfiguration.h>	

// Own Libraries
#include "../libraries/VectorMath/VectorMath.h"
#include "../libraries/PID/PID.h"
#include "../libraries/StateVariables/StateVariables.h"
#include "../libraries/TrajectoryControl/TrajectoryControl.h"
#include "../libraries/StandardController/StandardController.h"
#include "../libraries/Path/Trajectory.h"							// Trajectory design
#include "../libraries/Trajectory_management/Acceleration_mgt.h"	// maybe improve comments
#include "../libraries/Interface/Interface.hpp"						// Not (yet) in use
#include "../libraries/API_perso/API_perso.h"						// Not (yet) in use
#include "generateOutSignals.h"
#include "../libraries/StateVariablesEstimation/StateVariablesEstimation.h"
#include "../libraries/PathDelay/PathDelay.h"
//#include "../libraries/API_perso/API_perso.h"

//***************************************************
// Variable Declaration
//***************************************************

//Flying state
int Plane_flying_current_state=takeoff_mode;
int Plane_flying_next_state=takeoff_mode;
BOOL plane_flying_busy=FALSE;

// Position variables
struct vector pathned;	// desired position of the airplane
struct vector trajectory_refgnd; // Distance vector between the desired position and current position of the airplane

// Steering signals container
struct SteeringSignals stSig;

// Variables for Aerobatic Trajectory Controller
TrajectoryController trCTRL (hal,PERIOD); 
float phiRef=0;
struct vector aCMD_refin, gCMD_refin = GRAVITY_NED, aCMD_refbody, gCMD_refbody;
struct StateVariables stateVars, prevStateVars;

// Variables for Throttle error computation
PathDelay pathDly;
float desiredL = LookAheadDistance;
float errorThrottle;

// Interface
Interface intface(hal);
char consoleInRaw[size_buffer_interface];

//time management
uint32_t relative_time = hal.scheduler->micros(); //time from the last initialisation
uint32_t zero_time = hal.scheduler->micros(); //time of the last initialisation
uint32_t hardware_time = hal.scheduler->micros(); // real hadware time
uint32_t timer = 0; // trajectory timer


//temp for test							<--------------------------------------- temp
StateVariables copyOfStateVars;
uint32_t resetGPS_time;
BOOL testLock,testLock2;

//***************************************************
// Setup cycle
//***************************************************

// setup: called once at boot
void setup()
{
    Wire.begin(); // Begin I2C communcation
    int i;
    // Initialize dataSample to all 0
    for(i = 0; i < DATAPOINTS; i++) {
        dataSample.data.f[i] = 0.0;
    }

    // Trajectory Controller: Make all settings for the Aerobatic Trajectory Controller
    setupTrCTRL(trCTRL);

    // Enable PWM output on channels 0 to 3
    hal.rcout->enable_ch(0);
    hal.rcout->enable_ch(1);
    hal.rcout->enable_ch(2);
    hal.rcout->enable_ch(3);
    hal.rcout->enable_ch(4); // Enabling the rudder

    // Set next times for PWM output, serial output, and target change
    nextWrite = hal.scheduler->micros() + PERIOD;
    nextPrint = hal.scheduler->micros() + 1000000;
}


//***************************************************
// Loop Cycle
//***************************************************

// loop: called repeatedly in a loop
void loop()
{
    hardware_time = hal.scheduler->micros(); // real hadware time
    relative_time=hardware_time-zero_time; //time since the new simulation

    /* zero space time:
    Manage the plane crash or reinitialsiation*/
    if( ( fabs(stateVars.pnPePd.x-center_zero_space_x) < zero_space_size )
    &&  ( fabs(stateVars.pnPePd.y-center_zero_space_y) < zero_space_size )
    &&  ( fabs(stateVars.pnPePd.z-center_zero_space_z) < zero_space_size )
    &&  ( relative_time > time_before_retesting_zero_time_condition ) )
    {

        //time initialisation
        relative_time = 0; //will be recalculate at each iteration
        zero_time=hal.scheduler->micros();

        firstLoop=TRUE;
    }
    //----//

    if(hardware_time >= nextWrite) {
        nextWrite += PERIOD;

        // Read sensor data from emulation adapter
        uint8_t size = Wire.requestFrom(2, sizeof(dataSample.data));
        int i = 0;
        while(Wire.available()) {
            dataSample.data.raw[i] = Wire.read();
            i++;
        }


        //***************************************************//
        // Interface

        // Read console data from COM-PORT. Only -size_buffer_interface- characters allowed
        i = 0; //init
        while ( hal.console->available()
        AND    i < size_buffer_interface ) 
        {
            consoleInRaw[i] = hal.console->read();
//            hal.console->printf("%c",i);
            i++;
        }
        consoleInRaw[i] = '\0';
        if(i!=0) API_interpretate_chain(consoleInRaw, min(0,i-1),trCTRL, Plane_flying_current_state); //i=0 means that there is nothing in the buffer
        
        /*// go to the Interface-Handler
        if (consoleInRaw[0]!='\0'){
            interface.update(consoleInRaw);
        }*/

        //***************************************************
        // Measurements of the plane

        // From the measured data of the plane, calculate all necessary state variables.
        stateVars = calculateStateVariables (dataSample);
		
		
        //***************************************************
        // Process a loss of GPS-signal: 
        //If (ideal) Euler angles indicate a loss of GPS signal, stateVars will be overwritten by iterative estimating method as long as (ideal) Euler angles are in a GPS-losing position.

        // Check for GPS-signal loss
        if(firstLoop){
            prevStateVars.importData(stateVars);
            resetGPS_time = hal.scheduler->micros()-zero_time;
        }
    copyOfStateVars.importData(stateVars);

        if (1/* NoSignalAvailableGPS(stateVars.phiThetaPsi, 60) */) {
                // If signal loss, process new state variables with iteration method
                estimateStateVars (hardware_time,hal,stateVars, prevStateVars);	// Because stateVars already contains the ideal values, only "real measured" data will be taken. The rest will be overwritten by estimating method.
        }
        // Save stateVars for possible iteration in next time step
        prevStateVars.importData(stateVars);

    stateVars.importData(copyOfStateVars);

    if(relative_time>=resetGPS_time){
            resetGPS_time = hal.scheduler->micros()-zero_time+5e6;
            //hal.console->printf("ResetGPS.\n");
            prevStateVars.importData(stateVars);
    }
        //***************************************************
        // Path management
        if(firstLoop){
            pathned.x = center_zero_space_x;
            pathned.y = center_zero_space_y;
            pathned.z = center_zero_space_z;
            phiRef = 0;
            testLock = FALSE;
            testLock2 = FALSE;
        }
        if(plane_flying_busy==FALSE){
         Plane_flying_current_state=Plane_flying_next_state; //if the plane is not busy, we change the state
          timer = relative_time;
          }
        switch(Plane_flying_current_state)
            {
            case takeoff_mode:
                plane_flying_busy=TRUE;
                //code here
                trajectory_refgnd = traj_takeoff(pathned, stateVars.pnPePd);
                if(relative_time-timer > takeoff_time){
                    plane_flying_busy=FALSE;
                    timer = relative_time;
                    if(Plane_flying_next_state==Plane_flying_current_state) Plane_flying_next_state=glide_mode; //security
                    }
                break;
            case circle_mode:
                //code
                trajectory_refgnd = traj_circle(pathned, stateVars.pnPePd, relative_time);
                if(relative_time-timer > circle_time){
                    plane_flying_busy=FALSE;
                    timer = relative_time;
                    if(Plane_flying_next_state==Plane_flying_current_state) Plane_flying_next_state=glide_mode; //security
                    }
                break; 
            case looping_mode:
                plane_flying_busy=TRUE;
                //code here
                trajectory_refgnd = traj_loop(pathned, stateVars.pnPePd, relative_time, timer);
                if(relative_time-timer > looping_duration){ // need reconsider, in accordance with the looping func
                    plane_flying_busy=FALSE;
                    timer = relative_time;
                    if(Plane_flying_next_state==Plane_flying_current_state) Plane_flying_next_state=glide_mode; //security
                    }

                break;
            case glide_mode:
                //code
                trajectory_refgnd = traj_glide(pathned, stateVars.pnPePd);
                    plane_flying_busy=FALSE;
                    timer = relative_time;
                break; 
            case roll_mode:
                plane_flying_busy=TRUE;
                //code here
                trajectory_refgnd = traj_roll(pathned, stateVars.pnPePd);
                phiRef += 180.0/5e6*static_cast<float>(PERIOD);
                if (360>phiRef>180 || testLock==TRUE){
                    phiRef=180;
                    testLock=TRUE;
                }
                if (phiRef>360 || testLock2==TRUE){
                phiRef=0;
                testLock2=TRUE;
                }
                if(relative_time-timer > rolling_time){
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
/*
        //***************************************************
        // Path Generation
        if(firstLoop){
            pathned.x = center_zero_space_x;
            pathned.y = center_zero_space_y;
            pathned.z = center_zero_space_z;
            phiRef = 0;
            testLock = FALSE;
            testLock2 = FALSE;
        }
        if (relative_time<30e6){
            pathned.x += 0.0 * static_cast<float>(PERIOD) /1e6;
            pathned.y += 40.0 * static_cast<float>(PERIOD) /1e6;
            pathned.z += -15.0 * static_cast<float>(PERIOD) /1e6;
        }else if (relative_time<50e6){
            pathned.x += 0.0 * static_cast<float>(PERIOD) /1e6;
            pathned.y += 50.0 * static_cast<float>(PERIOD) /1e6;
            pathned.z += -5.0 * static_cast<float>(PERIOD) /1e6;

        }else if (relative_time<60e6){
        pathned.x += 0.0 * static_cast<float>(PERIOD) /1e6;
        pathned.y += 50.0 * static_cast<float>(PERIOD) /1e6;
        pathned.z += -0.0 * static_cast<float>(PERIOD) /1e6;
        phiRef += 180.0/5e6*static_cast<float>(PERIOD);
        if (phiRef>180 || testLock==TRUE){
            phiRef=180;
            testLock=TRUE;
        }

        }else{
            pathned.x += 0.0 * static_cast<float>(PERIOD) /1e6;
            pathned.y += 50.0 * static_cast<float>(PERIOD) /1e6;
            pathned.z += 0.0 * static_cast<float>(PERIOD) /1e6;
            phiRef += 180.0/5e6*static_cast<float>(PERIOD);
            if (phiRef>360 || testLock2==TRUE){
                phiRef=0;
                testLock2=TRUE;
            }
        }

*/
//        trajectory_refgnd = FlyTrajectory(firstLoop, pathned, stateVars.pnPePd, relative_time, tstart, tend);

        //***************************************************

        // Path Processing
        // Calculating the differential Trajectory
        //trajectory_refgnd = subtractVector(pathned,stateVars.pnPePd);

         // Calculating controller input for throttle
        errorThrottle = pathDly.update(pathned,trajectory_refgnd);
        errorThrottle -= desiredL;

        // Calculating the Acceleration out of Position Difference L
        aCMD_refin = Get_Acc_straigth(hal,stateVars.pnPePdDot,trajectory_refgnd);

        // Access the control structure
        aCMD_refbody = NEDtoBODY (aCMD_refin, stateVars.phiThetaPsi);
        gCMD_refbody = NEDtoBODY (gCMD_refin, stateVars.phiThetaPsi);


        //***************************************************
        // Controller
        gCMD_refbody = multiplyScalarToVector(gCMD_refbody,0.2);
        aCMD_refbody = subtractVector(aCMD_refbody,gCMD_refbody);
        stSig = trCTRL.update(hardware_time,errorThrottle,aCMD_refbody,gCMD_refbody,stateVars,phiRef);
        stSig.rudder = 0;

        //***************************************************
        // Print to Console

        // Printing in 20ms cycle
        if (firstLoop){
            hal.console->printf("eEulerDot.x\teEulerDot.y\teEulerDot.z\teEuler.x\teEuler.y\teEuler.z\teAcc.x\teAcc.y\teAcc.z\tePQRxUVW.x\tePQRxUVW.y\tePQRxUVW.z\teV.x\teV.y\teV.z\teVb.x\teVb.y\teVb.z\teXYZ.x\teXYZ.y\teXYZ.z\t");
            hal.console->printf("rEulerDot.x\trEulerDot.y\trEulerDot.z\trEuler.x\trEuler.y\trEuler.z\trAcc.x\trAcc.y\trAcc.z\trPQRxUVW.x\trPQRxUVW.y\trPQRxUVW.z\trV.x\trV.y\trV.z\trVb.x\trVb.y\trVb.z\trXYZ.x\trXYZ.y\trXYZ.z\n");
            //hal.console->printf("VxL.x,VxL.y,VxL.z,normL,errorAileron,errorRudder,errorElevator,errorThrottle,P_des.x,P_des.y,P_des.z,P_is.x,P_is.y,P_is.z,L_is.x,L_is.y,L_is.z,aCMDin.x,aCMDin.y,aCMDin.z,aCMDb.x,aCMDb.y,aCMDb.z,ACMDb.x,ACMD.y,ACMDb.z\n");
        }
        //hal.console->printf(",%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",pathned.x,pathned.y,pathned.z,stateVars.pnPePd.x,stateVars.pnPePd.y,stateVars.pnPePd.z,trajectory_refgnd.x,trajectory_refgnd.y,trajectory_refgnd.z,aCMD_refin.x,aCMD_refin.y,aCMD_refin.z,aCMD_refbody.x+gCMD_refbody.x,aCMD_refbody.y+gCMD_refbody.y,aCMD_refbody.z+gCMD_refbody.z,aCMD_refbody.x,aCMD_refbody.y,aCMD_refbody.z);

        // Printing in 1 sec cycle
        if(hardware_time >= nextPrint) {
            // Print some status
            nextPrint += 1000000;
            //hal.console->printf("** PERIOD **\r\n");
            // Print some values to the screen
                hal.console->printf("pathned: (%f,%f,%f)\npnPePd: (%f,%f,%f)\nL-vec: (%f,%f,%f)\n",pathned.x,pathned.y,pathned.z,stateVars.pnPePd.x,stateVars.pnPePd.y,stateVars.pnPePd.z,trajectory_refgnd.x,trajectory_refgnd.y,trajectory_refgnd.z);
                // Testwise printing the console-read variables
                //hal.console->printf("Read from COM-PORT: %c\n",consoleInRaw);
                //hal.console->printf("A:(%f,%f,%f), a:(%f,%f,%f), out:%f\n",aCMD_refbody.x,aCMD_refbody.y,aCMD_refbody.z,aCMD_refbody.x+gCMD_refbody.x,aCMD_refbody.y+gCMD_refbody.y,aCMD_refbody.z+gCMD_refbody.z,stSig.elevator);
                //hal.console->printf("aCMDb: (%f,%f,%f),\ngCMDb: (%f,%f,%f)\n",aCMD_refbody.x,aCMD_refbody.y,aCMD_refbody.z,gCMD_refbody.x,gCMD_refbody.y,gCMD_refbody.z);
                //hal.console->printf("euler: (%f,%f,%f)\n",stateVars.phiThetaPsi.x,stateVars.phiThetaPsi.y,stateVars.phiThetaPsi.z);
                //hal.console->printf("aCMDinertial: (%f,%f,%f)\n",aCMD_refin.x,aCMD_refin.y,aCMD_refin.z);
                //hal.console->printf("PID-In: %f, PID-Throttle out: %f, error-term: %f\n",inputThrottlePID,stSig.throttle,inputThrottlePID+stateVars.groundSpeed);
                //hal.console->printf("phi: %f, out: %f\n",stateVars.phiThetaPsi.z, stSig.rudder);
                //hal.console->printf("L-vec: (%f,%f,%f), speed: (%f,%f,%f),\naCMDin: (%f,%f,%f)\naCMDb: (%f,%f,%f), A_CMDb: (%f,%f,%f)\n\n",trajectory_refgnd.x,trajectory_refgnd.y,trajectory_refgnd.z,stateVars.pnPePdDot.x,stateVars.pnPePdDot.y,stateVars.pnPePdDot.z,aCMD_refin.x,aCMD_refin.y,aCMD_refin.z,aCMD_refbody.x+gCMD_refbody.x,aCMD_refbody.y+gCMD_refbody.y,aCMD_refbody.z+gCMD_refbody.z,aCMD_refbody.x,aCMD_refbody.y,aCMD_refbody.z);
                //hal.console->printf("%f\t%f\t%f\n",inputThrottlePID-stateVars.groundSpeed,inputThrottlePID,stateVars.groundSpeed);
                //hal.console->printf("in_velocity: %f\tbo_velocity: %f\n",velocity(stateVars.pnPePdDot),stateVars.groundSpeed);
                //hal.console->printf("uvw: (%f,%f,%f)\tpos_in: (%f,%f,%f)\n\n",stateVars.uvw.x,stateVars.uvw.y,stateVars.uvw.z,stateVars.pnPePd.x,stateVars.pnPePd.y,stateVars.pnPePd.z);
                //hal.console->printf("acc: (%f,%f,%f)\n",stateVars.accelerationBodyFrame.x,stateVars.accelerationBodyFrame.y,stateVars.accelerationBodyFrame.z);
                //hal.console->printf("DeltaL: %f\n",errorThrottle);
         /*   vector x = CrossProduct(stateVars.pqr,stateVars.uvw);
            hal.console->printf("%f\t%f\t%f\t",stateVars.phiThetaPsiDot.x,stateVars.phiThetaPsiDot.y,stateVars.phiThetaPsiDot.z);
            hal.console->printf("%f\t%f\t%f\t",stateVars.phiThetaPsi.x,stateVars.phiThetaPsi.y,stateVars.phiThetaPsi.z);
            hal.console->printf("%f\t%f\t%f\t",stateVars.pnPePdDotDot.x,stateVars.pnPePdDotDot.y,stateVars.pnPePdDotDot.z);
            hal.console->printf("%f\t%f\t%f\t",x.x,x.y,x.z);
            hal.console->printf("%f\t%f\t%f\t",stateVars.pnPePdDot.x,stateVars.pnPePdDot.y,stateVars.pnPePdDot.z);
            hal.console->printf("%f\t%f\t%f\t",stateVars.uvw.x,stateVars.uvw.y,stateVars.uvw.z);
            hal.console->printf("%f\t%f\t%f\n",stateVars.pnPePd.x,stateVars.pnPePd.y,stateVars.pnPePd.z);

     */ //  hal.console->printf("task-time: %i\n",-hardware_time+hal.scheduler->micros());
        }


        //***************************************************
        // Generate and Send output signals

        //generate Outupt signals
        generateOutSignals(stSig, aileronLOut,aileronROut,elevatorLOut,elevatorROut,throttleOut,rudderOut);

        // Output PWM
        hal.rcout->write(0, throttleOut);
        hal.rcout->write(1, elevatorLOut);
        hal.rcout->write(2, aileronROut);
        hal.rcout->write(3, aileronLOut);
        hal.rcout->write(4, rudderOut);   // Rudder output
		
        // Switch off first Loop indicator
        if (firstLoop){
            firstLoop = 0;
        }
    }
}

AP_HAL_MAIN();
