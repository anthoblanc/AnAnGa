/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil 
#define THISFIRMWARE "CPS-Autopilot-Project"

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
#include <PID.h>
#include <formulasForStateVariables.h>

// Gains for pidHeading
#define Kp_Heading 1
#define Ki_Heading 0.0001
#define Kd_Heading 0.01

// Gains for pidRoll
#define Kp_Roll 1
#define Ki_Roll 0
#define Kd_Roll 0

// Gains for pidAltitude
#define Kp_Altitude 0.8
#define Ki_Altitude 0.01
#define Kd_Altitude 0.01

// Gains for pidClimbRate
#define Kp_ClimbRate 0.5
#define Ki_ClimbRate 0.0001
#define Kd_ClimbRate 0

// Gains for pidPitch
#define Kp_Pitch 0.5
#define Ki_Pitch 0.0001
#define Kd_Pitch 0.001

// Gains for pidSpeed
#define Kp_Speed 1
#define Ki_Speed 0.001
#define Kd_Speed 0.001

// Loop period in microseconds
#define PERIOD 20000

// Hardware Abstraction Layer
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// Declare PIDs
// NOTE: You need to implement your own PID class at ../libraries/PID/
struct PIDs {
    PIDcontroller Heading;
    PIDcontroller Roll;
    PIDcontroller Altitude;
    PIDcontroller ClimbRate;
    PIDcontroller Pitch;
    PIDcontroller Speed;
};
struct PIDs PID = { {Kp_Heading,Ki_Heading,Kd_Heading,PERIOD,3},
                        {Kp_Roll,Ki_Roll,Kd_Roll,PERIOD,1},
                        {Kp_Altitude,Ki_Altitude,Kd_Altitude,PERIOD,9},
                        {Kp_ClimbRate,Ki_ClimbRate,Kd_ClimbRate,PERIOD,3},
                        {Kp_Pitch,Ki_Pitch,Kd_Pitch,PERIOD,1},
                        {Kp_Speed,Ki_Speed,Kd_Speed,PERIOD,1}};



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
float targetHeading, targetAltitude, targetSpeed;

// Hard bounds on the pitch and roll of the plane
float hardMaxPitch, hardMaxRoll;

// Time of next PWM output
uint32_t nextWrite;

// Time of next serial output
uint32_t nextPrint;

// mod ensures that val is between min and max. This is mostly used
// for ensuring that angles are within the range -Pi to Pi.


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

// setup: called once at boot
void setup()
{
    hal.console->printf("\r\n\r\nStarting up\r\n\r\n");
    Wire.begin(); // Begin I2C communcation
    int i;
    // Initialize dataSample to all 0
    for(i = 0; i < DATAPOINTS; i++) {
        dataSample.data.f[i] = 0.0;
    }

    // Initial flight control targets, note altitude is down.
    targetHeading = 100.0*(M_PI/180.0);
    targetSpeed = 15;
    targetAltitude = -200;

    // Set maximum pitch and roll to 30 degrees.
    hardMaxPitch = 30.0*(M_PI/180.0);
    hardMaxRoll = 30.0*(M_PI/180.0);

    // Construct PIDs with gains
    // Done above!

    // Enable PWM output on channels 0 to 3
    hal.rcout->enable_ch(0);
    hal.rcout->enable_ch(1);
    hal.rcout->enable_ch(2);
    hal.rcout->enable_ch(3);

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

        // Assignment of all read values to the state variables
        uvw.x = dataSample.data.f[I_VX];
        uvw.y = dataSample.data.f[I_VY];
        uvw.z = dataSample.data.f[I_VZ];

        uvwDot.x = dataSample.data.f[I_AX];
        uvwDot.y = dataSample.data.f[I_AY];
        uvwDot.z = dataSample.data.f[I_AZ];

        pqr.x = dataSample.data.f[I_P];
        pqr.y = dataSample.data.f[I_Q];
        pqr.z = dataSample.data.f[I_R];

        phiThetaPsi.x = dataSample.data.f[I_PHI];
        phiThetaPsi.y = dataSample.data.f[I_THETA];
        phiThetaPsi.z = dataSample.data.f[I_PSI];

        pnPePd.x = dataSample.data.f[I_LAT];
        pnPePd.y = dataSample.data.f[I_LON];
        pnPePd.z = dataSample.data.f[I_ALT];

        // Calculate the missing state variables
        phiThetaPsiDot = derivativeAngularRate( pqr, phiThetaPsi );
        pnPePdDot = derivativeVelocity( uvw, phiThetaPsi );
        groundSpeed = velocity( uvw );
        groundSpeedDot = acceleration ( uvw, uvwDot );

        // Use hal.console to receive a new target
            // hal.console->read()???
            // targetHeading = read()...
            // ...

        // Compute error in heading, ensuring it is in the range -Pi to Pi
        if(targetHeading>M_PI){targetHeading = targetHeading - 2*M_PI;}
        if(targetHeading<-M_PI){targetHeading = targetHeading + 2*M_PI;}

        // Compute heading PID
        float headingPIDOut = PID.Heading.update(targetHeading-phiThetaPsi.z,
                                phiThetaPsiDot.z);

        // Constrain output of heading PID such that it is a valid target roll
        headingPIDOut = constrain(headingPIDOut,-hardMaxRoll,hardMaxRoll);
        // Compute roll PID
        rollPIDOut = PID.Roll.update(headingPIDOut - phiThetaPsi.x,
                                phiThetaPsiDot.x);

        // Compute altitude PID
        float altitudePIDOut = PID.Altitude.update(-targetAltitude + pnPePd.z,
                                -pnPePdDot.z);

        // Compute climb rate PID
        float climbRatePIDOut = PID.ClimbRate.update(altitudePIDOut +   
                                 pnPePdDot.z,0);

        // Constrain output of climb rate PID such that it is a valid target pitch
        climbRatePIDOut = constrain (climbRatePIDOut,-hardMaxPitch,
                            +hardMaxPitch);

        // Compute pitch PID
        pitchPIDOut = PID.Pitch.update(climbRatePIDOut - phiThetaPsi.y,
                            phiThetaPsiDot.y);

        // Compute speed PID
        speedPIDOut = PID.Speed.update(targetSpeed - groundSpeed,groundSpeedDot);

        // Constrain all control surface outputs to the range -1 to 1
        float aileronL = -1 * constrain(rollPIDOut, -1, 1);
        float aileronR = constrain(rollPIDOut, -1, 1);
        float elevatorL = constrain(pitchPIDOut, -1, 1);
        float elevatorR = constrain(pitchPIDOut, -1, 1);
        float throttle = constrain(speedPIDOut, -1, 1);

#define SERVO_MIN 1000 // Minimum duty cycle
#define SERVO_MID 1500 // Mid duty cycle
#define SERVO_MAX 2000 // Maximum duty cycle
        // Compute duty cycle for PWM output from generic control
        int16_t aileronLOut = ((aileronL+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
        int16_t aileronROut = ((aileronR+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
        int16_t elevatorLOut = ((elevatorL+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
        int16_t elevatorROut = ((elevatorR+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
        int16_t throttleOut = ((throttle+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
        if(time >= nextPrint) {
            // Print some status
            nextPrint += 1000000;
            hal.console->printf("** PERIOD **\r\n");
            // Print some values to the screen
            
            
            hal.console->printf("targetAltitude: %f\n",targetAltitude);
            hal.console->printf("altitude: %f\n", pnPePd.z);
            hal.console->printf("error: %f\n", -targetAltitude+pnPePd.z);
            hal.console->printf("PIDOut: %f\n",altitudePIDOut);
hal.console->printf("targetClimbRate: %f\n",altitudePIDOut);
            hal.console->printf("ClimbRate: %f\n", pnPePdDot.z);
            hal.console->printf("error: %f\n", altitudePIDOut+pnPePdDot.z);
            hal.console->printf("PIDOut: %f\n",climbRatePIDOut);
hal.console->printf("targetPitch: %f\n",climbRatePIDOut);
            hal.console->printf("Pitch: %f\n", phiThetaPsi.y);
            hal.console->printf("error: %f\n", climbRatePIDOut-phiThetaPsi.y);
            hal.console->printf("PIDOut: %f\n",pitchPIDOut);

hal.console->printf("Update Success: %f\n",PID.Altitude.updateSuccessRate());

        }
        // Output PWM
        hal.rcout->write(0, throttleOut);
        hal.rcout->write(1, elevatorLOut);
        hal.rcout->write(2, aileronROut);
        hal.rcout->write(3, aileronLOut);
    }
}

AP_HAL_MAIN();
