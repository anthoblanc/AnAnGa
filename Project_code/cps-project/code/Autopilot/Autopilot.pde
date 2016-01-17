/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil 
#define THISFIRMWARE "CPS-Autopilot-Project"

/* ######################
         Library
   ###################### */

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


// Loop period in microseconds
#define PERIOD 20000

// Hardware Abstraction Layer
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// Number of data points sent by the simulator
#define DATAPOINTS 15

// Indices into array of data. Example: sample.data.f[I_PSI] gives you
// the heading.
#define I_AX 0
#define I_AY 1
#define I_AZ 2
#define I_P 3
#define I_Q 4
#define I_R 5
#define I_PHI 6
#define I_THETA 7
#define I_PSI 8
#define I_LAT 9
#define I_LON 10
#define I_ALT 11
#define I_VX 12
#define I_VY 13
#define I_VZ 14

// Data structure for each data sample. The data union is an array
// that can be accessed by bytes or by floats. Float access is for
// actually using data points. Byte access is for use by I2C code.
struct sample {
    union {
        float f[DATAPOINTS];
        uint8_t raw[DATAPOINTS * sizeof(float)];
    } data;
};

// Data storage
struct sample dataSample;

// Control targets
struct ControlTargets {
	float heading, altitude, speed;
};
struct ControlTargets target;

// Hard bounds on the pitch and roll of the plane
struct HardBounds {
	float maxPitch, maxRoll;
};
struct HardBounds hardBound;

// Time of next PWM output
uint32_t nextWrite;

// Time of next serial output
uint32_t nextPrint;

// mod ensures that val is between min and max. This is mostly used
// for ensuring that angles are within the range -Pi to Pi.

// Path start end time
uint32_t tstart = 30000000; // path starting time
uint32_t tend = 50000000; // path ending time

// constrain bounds val to at least min and at most max.
float constrain(float val, float min, float max)
{
    if(val < min) {
        return min;
    } else if(val > max) {
        return max;
    } else {
        return val;
    }
}

// Own Libraries
#include <PID.h>
#include <formulasForStateVariables.h>
#include <TrajectoryControl.h>
#include <StandardController.h>
#include <Path.h>
#include "../libraries/Trajectory_management/Acceleration_mgt.h"

struct vector trajectory_refgnd; //contain the direction of the path "L"

// temporary variables
uint8_t firstLoop = 1;

// Construct Standard Controller
StandardController stdCTRL(hal);
struct SteeringSignals stSig;

// Construct the Aerobatic Trajectory Controller
TrajectoryController trCTRL (hal,PERIOD,CO_Freq_LPF);

// Variables for reading the hal.console  <------- Testwise!
uint8_t consoleInRaw[21];   // limit to 20 characters

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
	
	// Set Constrains to flight manouvers and define Flight directions
	stdCTRL.setup(target, hardBound);
	
	// Make all settings for the Aerobatic Trajectory Controller
	setupTrCTRL(trCTRL);

    // Enable PWM output on channels 0 to 3
    hal.rcout->enable_ch(0);
    hal.rcout->enable_ch(1);
    hal.rcout->enable_ch(2);
    hal.rcout->enable_ch(3);
    //hal.rcout->enable_ch(5); // Enabling the rudder

    // Set next times for PWM output, serial output, and target change
    nextWrite = hal.scheduler->micros() + PERIOD;
    nextPrint = hal.scheduler->micros() + 1000000;
    
}

// loop: called repeatedly in a loop
void loop()
{
    uint32_t time = hal.scheduler->micros();
    if(time >= nextWrite) {
        nextWrite += PERIOD;

        // Read sensor data from emulation adapter
        uint8_t size = Wire.requestFrom(2, sizeof(dataSample.data));
        int i = 0;
        while(Wire.available()) {
            dataSample.data.raw[i] = Wire.read();
            i++;
        }

        // Read console data from COM-PORT. Only 20 characters allowed
        /*i = 0;
        while(hal.console->available() && i<20) {
            consoleInRaw[i] = hal.console->read();
            i++;
        }
        consoleInRaw[20] = '\0';
        */
		
            // Updating the StandardController
            //stSig = stdCTRL.update(dataSample, target, hardBound);
		
		
            // Updating the Aerobatic Trajectory Controller
            float deltaL, phiRef=0;
            struct vector aCMD_refin, gCMD_refin = {0,0,-9.81}, aCMD_refbody, gCMD_refbody, eulerDesired = {0,0,0};
            int8_t aerobatOn = 0;

            // From the measured data of the plane, calculate all necessary state variables.
            struct StateVariables stateVars;
            stateVars = calculateStateVariables (dataSample);
			
			
			
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

            // Testwise desired Trajectory
            if (firstLoop){
                pathned.x = stateVars.pnPePd.x;
                pathned.y = stateVars.pnPePd.y;
                pathned.z = stateVars.pnPePd.z;
                //firstLoop = 0; Switched off down at printing!
            }
            if (time<30e6){
                pathned.x += 0 * PERIOD/1e6;
                pathned.y += 15 * PERIOD/1e6;
                pathned.z += 3 * PERIOD/1e6;
            }else{
                pathned.x += 0 * PERIOD/1e6;
                pathned.y += 15 * PERIOD/1e6;
                pathned.z += 0 * PERIOD/1e6;
            }

            // Calculating differential Trajectory
            trajectory_refgnd.x = pathned.x - stateVars.pnPePd.x;
            trajectory_refgnd.y = pathned.y - stateVars.pnPePd.y;
            trajectory_refgnd.z = pathned.z - stateVars.pnPePd.z;

            // Calculating Delta L
            float desiredL = 10;    // Testwise (could depend from velocity)
            deltaL = desiredL - NormVector(trajectory_refgnd);

            // Calculating the Acceleration out of
            aCMD_refin = Get_Acc_straigth(stateVars.pnPePdDot,trajectory_refgnd);

            // Access the control structure
            aCMD_refbody = NEDtoBODY (aCMD_refin, stateVars.phiThetaPsi);
            gCMD_refbody = NEDtoBODY (gCMD_refin, stateVars.phiThetaPsi);
            stSig = trCTRL.update(deltaL,aCMD_refbody,gCMD_refbody,phiRef,aerobatOn,eulerDesired);

			
		
        // Constrain all control surface outputs to the range -1 to 1
        float aileronL = -1 * constrain(stSig.aileron, -1, 1);
        float aileronR = constrain(stSig.aileron, -1, 1);
        float elevatorL = constrain(stSig.elevator, -1, 1);
        float elevatorR = constrain(stSig.elevator, -1, 1);
        float throttle = constrain(stSig.throttle, -1, 1);
        float rudder = constrain(stSig.rudder, -1, 1);

        #define SERVO_MIN 1000 // Minimum duty cycle
        #define SERVO_MID 1500 // Mid duty cycle
        #define SERVO_MAX 2000 // Maximum duty cycle
        // Compute duty cycle for PWM output from generic control
        int16_t aileronLOut = ((aileronL+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
        int16_t aileronROut = ((aileronR+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
        int16_t elevatorLOut = ((elevatorL+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
        int16_t elevatorROut = ((elevatorR+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
        int16_t throttleOut = ((throttle+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
        int16_t rudderOut = ((rudder+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;

        // Printing in 20mx cycle
        if (firstLoop){
            hal.console->printf("L_is.x,L_is.y,L_is.z,Delta_L,aCMDin.x,aCMDin.y,aCMDin.z,aCMDb.x,aCMDb.y,aCMDb.z,Throttle,Aileron,Rudder,Elevator\n");
            firstLoop = 0; //Switch off at temp path when this here is removed!
        }
        hal.console->printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",trajectory_refgnd.x,trajectory_refgnd.y,trajectory_refgnd.z,deltaL,aCMD_refin.x,aCMD_refin.y,aCMD_refin.z,aCMD_refbody.x,aCMD_refbody.y,aCMD_refbody.z,stSig.throttle,stSig.aileron,stSig.rudder,stSig.elevator);

        // Printing in 1 sec cycle
        if(time >= nextPrint) {
            // Print some status
            nextPrint += 1000000;
            //hal.console->printf("** PERIOD **\r\n");
            // Print some values to the screen

                // Testwise printing the console-read variables
                //hal.console->printf("Read from COM-PORT: %s\n",consoleInRaw);
	
		//hal.console->printf("\naCMDb: (%f,%f,%f),\ngCMDb: (%f,%f,%f)\n",aCMD_refbody.x,aCMD_refbody.y,aCMD_refbody.z,gCMD_refbody.x,gCMD_refbody.y,gCMD_refbody.z);
                //hal.console->printf("Position: (%f,%f,%f)\n",stateVars.pnPePd.x,stateVars.pnPePd.y,stateVars.pnPePd.z);
		//hal.console->printf("\naCMDinertial: (%f,%f,%f)\n",aCMD_refin.x,aCMD_refin.y,aCMD_refin.z);
        }

        // Output PWM
        hal.rcout->write(0, throttleOut);
        hal.rcout->write(1, elevatorLOut);
        hal.rcout->write(2, aileronROut);
        hal.rcout->write(3, aileronLOut);
        //hal.rcout->write(5, rudderOut);   // Rudder output
    }
}

AP_HAL_MAIN();
