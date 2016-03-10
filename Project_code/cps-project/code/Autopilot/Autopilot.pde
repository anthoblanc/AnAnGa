/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil
#define THISFIRMWARE "CPS-Autopilot-Project"

//***************************************************
//                   Relevant Defines
//***************************************************

// Look Ahead Distance (Desired Length) [feet]
#define LookAheadDistance 40

// Activate the GPS-Signal Estimation when setting to 1
#define ActivateGPS_SignalEstimation 0

// Trajectory Settings
// -> in Trajectory.cpp

// StateVariablesEstimation: radius Coefficient Matrix
// -> In VectorMath.c

// Trajectory Controller: All Settings of the controllers
// -> in TrajectoryControl.h

// API: All setting concerning the interface over the COM-Port
// -> in API_Perso.cpp

//***************************************************
//***************************************************


//interface
#define size_buffer_interface 20

//General
typedef int           BOOL;
#define FALSE         0
#define TRUE          1
//Note: Boolean also exist in a library but we don't really need to do complex features

#define NOT           !
#define AND           &&
#define OR            ||


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
#include "../libraries/Path/Trajectory.h"							
#include "../libraries/Trajectory_management/Acceleration_mgt.h"	
#include "../libraries/API_perso/API_perso.h"						
#include "generateOutSignals.h"
#include "../libraries/StateVariablesEstimation/StateVariablesEstimation.h"
#include "../libraries/PathDelay/PathDelay.h"


//***************************************************
// Variable Declaration
//***************************************************

//Flying state
int Plane_flying_current_state=takeoff_mode;
int Plane_flying_next_state=takeoff_mode;
int Plane_flying_previous_state=takeoff_mode;
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
char consoleInRaw[size_buffer_interface];

//time management
uint32_t relative_time = hal.scheduler->micros(); //time from the last initialisation
uint32_t zero_time = hal.scheduler->micros(); //time of the last initialisation
uint32_t hardware_time = hal.scheduler->micros(); // real hadware time
uint32_t timer = 0; // trajectory timer

//temp for test							<--------------------------------------- temp
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

    if(hardware_time >= nextWrite) {
        nextWrite += PERIOD;

		//***************************************************
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
            Plane_flying_current_state=takeoff_mode;
            Plane_flying_next_state=takeoff_mode;
			Plane_flying_previous_state=takeoff_mode;
            plane_flying_busy=FALSE;
            firstLoop=TRUE;
        }


		//***************************************************
        // Read sensor data from emulation adapter
        uint8_t size = Wire.requestFrom(2, sizeof(dataSample.data));
        int i = 0;
        while(Wire.available()) {
            dataSample.data.raw[i] = Wire.read();
            i++;
        }


        //***************************************************
        // User Interface

        // Read console data from COM-PORT. Only -size_buffer_interface- characters allowed
        i = 0; //init
        while ( hal.console->available()
        AND    i < size_buffer_interface ) 
        {
            consoleInRaw[i] = hal.console->read();
            i++;
        }
        consoleInRaw[i] = '\0';
        if(i!=0) API_interpretate_chain(consoleInRaw, min(0,i-1),trCTRL, Plane_flying_next_state, desiredL); //i=0 means that there is nothing in the buffer

		
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
        }

        if ( NoSignalAvailableGPS(stateVars.phiThetaPsi, 60) && ActivateGPS_SignalEstimation) {
                // If signal loss, process new state variables with iteration method
                estimateStateVars (hardware_time,hal,stateVars, prevStateVars);	// Because stateVars already contains the ideal values, only "real measured" data will be taken. The rest will be overwritten by estimating method.
        }
        // Save stateVars for possible iteration in next time step
        prevStateVars.importData(stateVars);

		
        //***************************************************
        // Path Generation

        trajectory_refgnd = choose_traj(hal,firstLoop, Plane_flying_current_state, Plane_flying_next_state, Plane_flying_previous_state, plane_flying_busy, pathned, stateVars.pnPePd , relative_time, timer, phiRef);
        // Path can also be replaced by a total designed trajectory in a function of time, out commentted in Trajectory.cpp
		

        //***************************************************
        // Path Processing

         // Calculating controller input for throttle
        errorThrottle = pathDly.update(pathned, trajectory_refgnd);
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
            // Prints only for the startup phase
        }

        // Printing in 1 sec cycle
        if(hardware_time >= nextPrint) {
            // Print some status
            nextPrint += 1000000;
            //hal.console->printf("** PERIOD **\r\n");
            // Print some values to the screen
            //hal.console->printf("current: %d\tnext: %d\tprev: %d\n", Plane_flying_current_state,Plane_flying_next_state,Plane_flying_previous_state);
            //hal.console->printf("task-time: %i\n",-hardware_time+hal.scheduler->micros());
            //hal.console->printf("is: (%f,%f,%f)\ttraj: (%f,%f,%f)\n",stateVars.pnPePd.x,stateVars.pnPePd.y,stateVars.pnPePd.z,pathned.x,pathned.y,pathned.z);
        }


        //***************************************************
        // Generate and Send output signals

        //generate Outupt signals and hand to simulation environment
        generateOutSignals(stSig, hal);
		
        // Switch off first Loop indicator
        if (firstLoop){
            firstLoop = 0;
        }
    }
}

AP_HAL_MAIN();
