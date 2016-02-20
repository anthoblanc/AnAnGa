#ifndef STANDARDCONTROLLER_H
#define STANDARDCONTROLLER_H

/***********************************************************

StandardControler.h
	Library containing the controller for flying at a given speed in a given heading and altitude. Not used when in trajectory flight. This library was for the first task of the project.

	Parameters to tune the controller are listed below and are meant for altering.	
	
	struct ControlTargets:
		Container for the three controller targets: heading, altitude, and speed.

	struct HardBounds:
		Containger for the physical restrictions on the airplane, the controller has to satisfy.
	
	struct PIDs:
		Container for all PIDs, the standard controller has.
		
	enum stdPIDs:
		List of all PIDs the standard controller has. Used for direct access to a specific PID-controller.

		
	class StandardController:
		Constructor: 
			Constructs the controller with the parameters listed below.
			hal: Reference to the console for accessing its function (printf,...)
						
		setup(): 
			Give the controller the control targets and hard bounds, listed below. With little effort, here is the possibility, to override the control targets or hard bounds during flight.
				
		update(): 
			Routine for updating the state varibles of the airplane and updating all controller outputs of the standard controller.
			dataSample: Data, containing the state Variables, given by the console.
			
		getStateVariables():
			Possibility to get reading access to the state variables from outside the standard controller class.
			
		getPIDaccess():
			Possibility to get direct access to the specific PIDs of the standard controller. Possibilities of this access -> see PID.h
			

************************************************************/

// ----------------------------------------------------------------------------------------
// Variables for tuning the Standard Controller

// Gains for pidHeading
#define Kp_Heading 1			// Proportional gain
#define Ki_Heading 0.0001		// Integral gain
#define Kd_Heading 0.1			// Derivative gain
#define Casc_Heading 3			// Factor to set a multiple of the update time the controller is updated

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

// Control Target
#define TargetHeading 100.0			// heading in degrees
#define TargetSpeed 15.0			// Speed in ft/s
#define TargetAltitude -100.0		// Altitude in ft

// Control Bounds
#define MaxPitch 30.0				// Bounds in degrees
#define MaxRoll 30.0

// ----------------------------------------------------------------------------------------


// Control targets
struct ControlTargets {
        float heading, altitude, speed;
};


// Hard bounds on the pitch and roll of the plane
struct HardBounds {
        float maxPitch, maxRoll;
};


// Declare PIDs
struct PIDs {
    PIDcontroller Heading;
    PIDcontroller Roll;
    PIDcontroller Altitude;
    PIDcontroller ClimbRate;
    PIDcontroller Pitch;
    PIDcontroller Speed;
};


// Enumeration of all Standard PIDs
enum stdPIDs {heading,roll,altitude,climbRate,pitch,speed};

						
// Class StandardController
class StandardController {

public:
        // Constructor
        StandardController( const AP_HAL::HAL& hal );

        // Instructions for the setup routine
        void setup(/* Possibility here, to override the control targets or hard bounds */);

        // Update Routine in the Main-Function
        struct SteeringSignals update (const struct sample dataSample);

        // Getting values of the state variables of the plane
        const struct StateVariables *getStateVariables ();

        // Access to the PIDs
        // const is for denying any change in the PIDs
        const PIDcontroller *getPIDAccess (enum stdPIDs pidName);


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
        struct StateVariables m_stateVars;

        struct ControlTargets target;	// control targets for the controller
        struct HardBounds hardBound;	// Physical bounds the controller has to be satisfied

};


#include "StandardController.c"

#endif
