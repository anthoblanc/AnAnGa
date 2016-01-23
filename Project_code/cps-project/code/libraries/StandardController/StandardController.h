#ifndef STANDARDCONTROLLER_H
#define STANDARDCONTROLLER_H


// Set the gains for all PIDS
// Gains for pidHeading
#define Kp_Heading 1
#define Ki_Heading 0.0001
#define Kd_Heading 0.1
#define Casc_Heading 3

// Gains for pidRoll
#define Kp_Roll 1
#define Ki_Roll 0
#define Kd_Roll 0
#define Casc_Roll 1

// Gains for pidAltitude
#define Kp_Altitude 2
#define Ki_Altitude 0.01
#define Kd_Altitude 0.1
#define Casc_Altitude 3

// Gains for pidClimbRate
#define Kp_ClimbRate 1
#define Ki_ClimbRate 0
#define Kd_ClimbRate 0
#define Casc_ClimbRate 3

// Gains for pidPitch
#define Kp_Pitch 1
#define Ki_Pitch 0
#define Kd_Pitch 0
#define Casc_Pitch 1

// Gains for pidSpeed
#define Kp_Speed 1
#define Ki_Speed 0
#define Kd_Speed 0
#define Casc_Speed 1


// Declare PIDs
struct PIDs {
    PIDcontroller Heading;
    PIDcontroller Roll;
    PIDcontroller Altitude;
    PIDcontroller ClimbRate;
    PIDcontroller Pitch;
    PIDcontroller Speed;
};

enum States {UVW,UVWdot,PQR,PhiThetaPsi,PhiThetaPsiDot,PnPePd,PnPePdDot};
						
// Class Standardcontroller
class StandardController {

public:

        // Constructor
        StandardController( const AP_HAL::HAL& hal );

        // Instructions for the setup routine
        void setup(struct ControlTargets& target, struct HardBounds& hardBound);

        // Update Routine in the Main-Function
        struct SteeringSignals update (const struct sample dataSample, const struct ControlTargets target, const struct HardBounds hardBound);

        // Getting values of the state variables of the plane
        struct vector getVector (const enum States x);
        float getGroundSpeed ();
        float getGroundSpeedDot ();

        // Access to the PIDs
        // const is for denying any change in the PIDs
        const struct PIDs *getPIDAccess ();


private:
        const AP_HAL::HAL& m_rHAL;	// reference to the console (for printing messages)

        // All PIDs of the controller
        struct PIDs PID = { {Kp_Heading,Ki_Heading,Kd_Heading,PERIOD,Casc_Heading},
                {Kp_Roll,Ki_Roll,Kd_Roll,PERIOD,Casc_Roll},
                {Kp_Altitude,Ki_Altitude,Kd_Altitude,PERIOD,Casc_Altitude},
                {Kp_ClimbRate,Ki_ClimbRate,Kd_ClimbRate,PERIOD,Casc_ClimbRate},
                {Kp_Pitch,Ki_Pitch,Kd_Pitch,PERIOD,Casc_Pitch},
                {Kp_Speed,Ki_Speed,Kd_Speed,PERIOD,Casc_Speed}};

        // Declaration of all state variables for controlling the vehicle
        struct vector uvw;
        struct vector uvwDot;
        struct vector pqr;
        struct vector phiThetaPsi;
        struct vector phiThetaPsiDot;
        struct vector pnPePd;
        struct vector pnPePdDot;
        float groundSpeed;
        float groundSpeedDot;

};


#include "StandardController.c"

#endif
