/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil


//***************************************************
// Define
//***************************************************
#define THISFIRMWARE "CPS-Autopilot-Project"


//zeros time space zone
#define zero_space_size 5
#define center_zero_space_x -365
#define center_zero_space_y -400
#define center_zero_space_z -0.87


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

#include <StandardConfiguration.h>


// Own Libraries
#include "../libraries/VectorMath/VectorMath.h"
#include "../libraries/PID/PID.h"
#include "../libraries/StateVariables/StateVariables.h"
#include "../libraries/TrajectoryControl/TrajectoryControl.h"
#include "../libraries/StandardController/StandardController.h"
#include "../libraries/Path/Path.h"
#include "../libraries/Trajectory_management/Acceleration_mgt.h"
#include "../libraries/Interface/Interface.hpp"
#include "generateOutSignals.h"
#include "../libraries/StateVariablesEstimation/StateVariablesEstimation.h"

//***************************************************
// Variable Declaration
//***************************************************

struct vector trajectory_refgnd; //contain the direction of the path "L"

// Path start end time
uint32_t tstart = 20e6; // path starting time
uint32_t tend = 30e6; // path ending time

// temporary variables
uint8_t firstLoop = 1;

// Construct Standard Controller
//StandardController stdCTRL(hal);

struct SteeringSignals stSig;

// Construct the Aerobatic Trajectory Controller
TrajectoryController trCTRL (hal,PERIOD,CO_Freq_LPF);
float phiRef=0;
struct vector aCMD_refin, gCMD_refin = GRAVITY_NED, aCMD_refbody, gCMD_refbody, eulerDesired = {0,0,0};
int8_t aerobatOn = 0;
struct StateVariables stateVars, prevStateVars;

// Variables for Throttle error computation
struct vector prevPathNED = {-365,-400,0.87};
struct vector pathVelocity;
float inputThrottlePID;

// Interface
Interface intface(hal);

//time management
uint32_t relative_time=hal.scheduler->micros(); //time from the last initialisation
uint32_t zero_time=hal.scheduler->micros(); //time of the last initialisation
uint32_t hardware_time = hal.scheduler->micros(); // real hadware time

//***************************************************
// Setup cycle
//***************************************************

// setup: called once at boot
void setup()
{
    //hal.console->printf("\r\n\r\nStarting up\r\n\r\n");
    Wire.begin(); // Begin I2C communcation
    int i;
    // Initialize dataSample to all 0
    for(i = 0; i < DATAPOINTS; i++) {
        dataSample.data.f[i] = 0.0;
    }
	
    // Initial filling of state variables plus previous state variables
    // Read sensor data from emulation adapter
    while(Wire.available()) {
            dataSample.data.raw[i] = Wire.read();
            i++;
    }
    stateVars = calculateStateVariables (dataSample);
    prevStateVars.importData (stateVars);

    // StandardController: Set Constrains to flight manouvers and define Flight directions
    //stdCTRL.setup();

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
    relative_time=hardware_time-zero_time;

    /* zero space time:
    Manage the plane crash or reinitialsiation*/
    if( ( fabs(stateVars.pnPePd.x-center_zero_space_x) < zero_space_size )
    &&  ( fabs(stateVars.pnPePd.y-center_zero_space_y) < zero_space_size )
    &&  ( fabs(stateVars.pnPePd.z-center_zero_space_z) < zero_space_size )
    &&  ( relative_time > 10e6 ) )
    {
        firstLoop = 1;
        relative_time = 0;
        zero_time=hal.scheduler->micros();
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


        //***************************************************
        // Interface

        // Read console data from COM-PORT. Only 20 characters allowed
        /*i = 0;
        consoleInRaw[0] = '\0';
        while(hal.console->available() && i<20) {
            consoleInRaw[i] = hal.console->read();
            i++;
        }
        consoleInRaw[20] = '\0';
        */// go to the Interface-Handler
        /*if (consoleInRaw[0]!='\0'){
            interface.update(consoleInRaw);
        }*/



        // Updating the StandardController
        //stSig = stdCTRL.update(dataSample);
        //stSig.rudder=0;


        //***************************************************
        // Measurements of the plane

        // From the measured data of the plane, calculate all necessary state variables.
        stateVars = calculateStateVariables (dataSample);
		
		
        //***************************************************
        // Process a loss of GPS-signal: If (ideal) Euler angles indicate a loss of GPS signal, stateVars will be overwritten by iterative estimating method as long as (ideal) Euler angles are in a GPS-losing position.

        // Check for GPS-signal loss
        if ( ~SignalCheckGPS(stateVars.phiThetaPsi, 60) ) {
                // If signal loss, process new state variables with iteration method
                //estimateStateVars (stateVars, prevStateVars);	// Because stateVars already contains the ideal values, only "real measured" data will be taken. The rest will be overwritten by estimating method.
        }
        // Save stateVars for possible iteration in next time step
        prevStateVars.importData(stateVars);

        //***************************************************
        // Path Generation

        /*	// Path Control
            // use two points before loop to decide direction
            if (hardware_time <tstart){     //
            trajectory_refgnd.x = 6;
            trajectory_refgnd.y = 80;
             trajectory_refgnd.z = -10;
            }
            if (hardware_time >= (tstart - 1500000) && hardware_time <= (tstart - 1000000) ){     //coordinate 1 sec before loop
            pnPePdtmp0 = stateVars.pnPePd;
            }
             if(hardware_time >= (tstart - 5000000) && hardware_time <= tstart) { // coordinate just before loop
             pnPePdtmp = stateVars.pnPePd;
             }
            // 20s -> 40s, perform loop, give target coordinates two secs in the future
            if(hardware_time >= tstart && hardware_time <= tend)
            {
                pathned = pathloop ( pnPePdtmp0, pnPePdtmp, hardware_time, tstart, tend );
                //calculate the deisred path "L"
                trajectory_refgnd.x = pathned.x - stateVars.pnPePd.x;
                trajectory_refgnd.y = pathned.y - stateVars.pnPePd.y;
                trajectory_refgnd.z = pathned.z- stateVars.pnPePd.z;
            }
        */
/*        if (firstLoop){
                pathned = traj_initialize(stateVars.pnPePd);
            }
          if (hardware_time<10e6){
            pathned.x += 0.0 *  static_cast<float>(PERIOD) /1e6;
            pathned.y += 100.0 * static_cast<float>(PERIOD) /1e6;
            pathned.z -= 20.0 * static_cast<float>(PERIOD) /1e6;

                trajectory_refgnd = traj_takeoff(pathned, stateVars.pnPePd, stateVars.pnPePd);
                }else if (hardware_time<20e6){
                pathned.x += 0.0 * static_cast<float>(PERIOD) /1e6;//* inertial_velocity.x * static_cast<float>(PERIOD) /1e6;
                pathned.y += 100.0 * static_cast<float>(PERIOD) /1e6;//* inertial_velocity.y * static_cast<float>(PERIOD) /1e6;
                pathned.z -= 40.0 * static_cast<float>(PERIOD) /1e6;//* inertial_velocity.z * static_cast<float>(PERIOD) /1e6;

                trajectory_refgnd = traj_climbup(pathned, stateVars.pnPePd, stateVars.pnPePd);
                }

            else if (hardware_time<30e6){
                trajectory_refgnd = traj_loop(pathned, stateVars.pnPePd, hardware_time, tstart, tend);
            }
            else {
            pathned.x += 0.0 * static_cast<float>(PERIOD) /1e6;//* inertial_velocity.x * static_cast<float>(PERIOD) /1e6;
            pathned.y += 100.0 * static_cast<float>(PERIOD) /1e6;//* inertial_velocity.y * static_cast<float>(PERIOD) /1e6;
            pathned.z -= 40.0 * static_cast<float>(PERIOD) /1e6;//* inertial_velocity.z * static_cast<float>(PERIOD) /1e6;

            trajectory_refgnd = traj_climbup(pathned, stateVars.pnPePd, stateVars.pnPePdDot);
            }
*/
        // Testwise desired Trajectory
        if (firstLoop){
            pathned.x = -365;
            pathned.y = -400;
            pathned.z = -0.87;
        }
        if (hardware_time<30e6){
            pathned.x += 0.0 * static_cast<float>(PERIOD) /1e6;
            pathned.y += 50.0 * static_cast<float>(PERIOD) /1e6;
            pathned.z += -5.0 * static_cast<float>(PERIOD) /1e6;
        }else if (hardware_time<50e6){
            pathned.x += 0.0 * static_cast<float>(PERIOD) /1e6;
            pathned.y += 50.0 * static_cast<float>(PERIOD) /1e6;
            pathned.z += -0.0 * static_cast<float>(PERIOD) /1e6;
        }else{
            pathned.x += 0.0 * static_cast<float>(PERIOD) /1e6;
            pathned.y += 50.0 * static_cast<float>(PERIOD) /1e6;
            pathned.z += 0.0 * static_cast<float>(PERIOD) /1e6;
        }
        //***************************************************
        // Path Processing
        // Calculating differential Trajectory
        trajectory_refgnd = subtractVector(pathned,stateVars.pnPePd);

        // Calculating controller input for throttle
        float desiredL = 50;    // [ft] Testwise (could depend from velocity)
        pathVelocity = multiplyScalarToVector ( subtractVector(pathned,prevPathNED), 1/(1.0*PERIOD/1e6) );
        inputThrottlePID = ScalarProduct(pathVelocity,trajectory_refgnd) / NormVector(pathVelocity);
        inputThrottlePID -= desiredL;
        inputThrottlePID *= 1.0/2.0;
        inputThrottlePID += stateVars.groundSpeed;
        prevPathNED.importVector(pathned);

        // Calculating the Acceleration out of Position Difference L
        aCMD_refin = Get_Acc_straigth(hal,stateVars.pnPePdDot,trajectory_refgnd);

        // Access the control structure
        aCMD_refbody = NEDtoBODY (aCMD_refin, stateVars.phiThetaPsi);
        gCMD_refbody = NEDtoBODY (gCMD_refin, stateVars.phiThetaPsi);


        //***************************************************
        // Controller

        aCMD_refbody = subtractVector(aCMD_refbody,gCMD_refbody);
        stSig = trCTRL.update(hardware_time,inputThrottlePID,aCMD_refbody,gCMD_refbody,stateVars,phiRef,aerobatOn,eulerDesired);


        //***************************************************
        // Print to Console

        // Printing in 20ms cycle
        if (firstLoop){
            //hal.console->printf("VxL.x,VxL.y,VxL.z,normL,errorAileron,errorRudder,errorElevator,errorThrottle,P_des.x,P_des.y,P_des.z,P_is.x,P_is.y,P_is.z,L_is.x,L_is.y,L_is.z,aCMDin.x,aCMDin.y,aCMDin.z,aCMDb.x,aCMDb.y,aCMDb.z,ACMDb.x,ACMD.y,ACMDb.z\n");
        }
        //hal.console->printf(",%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",pathned.x,pathned.y,pathned.z,stateVars.pnPePd.x,stateVars.pnPePd.y,stateVars.pnPePd.z,trajectory_refgnd.x,trajectory_refgnd.y,trajectory_refgnd.z,aCMD_refin.x,aCMD_refin.y,aCMD_refin.z,aCMD_refbody.x+gCMD_refbody.x,aCMD_refbody.y+gCMD_refbody.y,aCMD_refbody.z+gCMD_refbody.z,aCMD_refbody.x,aCMD_refbody.y,aCMD_refbody.z);

        // Printing in 1 sec cycle
        if(hardware_time >= nextPrint) {
            // Print some status
            nextPrint += 1000000;
            //hal.console->printf("** PERIOD **\r\n");
            // Print some values to the screen
                //hal.console->printf("pathned: (%f,%f,%f)\npnPePd: (%f,%f,%f)\n",pathned.x,pathned.y,pathned.z,stateVars.pnPePd.x,stateVars.pnPePd.y,stateVars.pnPePd.z);
                // Testwise printing the console-read variables
                //hal.console->printf("Read from COM-PORT: %c\n",consoleInRaw);
                //hal.console->printf("A:(%f,%f,%f), a:(%f,%f,%f), out:%f\n",aCMD_refbody.x,aCMD_refbody.y,aCMD_refbody.z,aCMD_refbody.x+gCMD_refbody.x,aCMD_refbody.y+gCMD_refbody.y,aCMD_refbody.z+gCMD_refbody.z,stSig.elevator);
                //hal.console->printf("aCMDb: (%f,%f,%f),\ngCMDb: (%f,%f,%f)\n",aCMD_refbody.x,aCMD_refbody.y,aCMD_refbody.z,gCMD_refbody.x,gCMD_refbody.y,gCMD_refbody.z);
                //hal.console->printf("euler: (%f,%f,%f)\n",stateVars.phiThetaPsi.x,stateVars.phiThetaPsi.y,stateVars.phiThetaPsi.z);
                //hal.console->printf("aCMDinertial: (%f,%f,%f)\n",aCMD_refin.x,aCMD_refin.y,aCMD_refin.z);
                //hal.console->printf("PID-In: %f, PID-Throttle out: %f, error-term: %f\n",inputThrottlePID,stSig.throttle,inputThrottlePID+stateVars.groundSpeed);
                //hal.console->printf("phi: %f, out: %f\n",stateVars.phiThetaPsi.z, stSig.rudder);
                //hal.console->printf("L-vec: (%f,%f,%f), speed: (%f,%f,%f),\naCMDin: (%f,%f,%f)\naCMDb: (%f,%f,%f), A_CMDb: (%f,%f,%f)\n\n",trajectory_refgnd.x,trajectory_refgnd.y,trajectory_refgnd.z,stateVars.pnPePdDot.x,stateVars.pnPePdDot.y,stateVars.pnPePdDot.z,aCMD_refin.x,aCMD_refin.y,aCMD_refin.z,aCMD_refbody.x+gCMD_refbody.x,aCMD_refbody.y+gCMD_refbody.y,aCMD_refbody.z+gCMD_refbody.z,aCMD_refbody.x,aCMD_refbody.y,aCMD_refbody.z);
                hal.console->printf("%f\t%f\t%f\n",inputThrottlePID-stateVars.groundSpeed,inputThrottlePID,stateVars.groundSpeed);
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
		
        // Switch off first Loop
        if (firstLoop){
            firstLoop = 0; //Switch off at temp path when this here is removed!
        }
    }
}

AP_HAL_MAIN();
