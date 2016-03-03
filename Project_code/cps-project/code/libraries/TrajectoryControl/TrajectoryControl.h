#ifndef TRAJECTORYCONTROL_H
#define TRAJECTORYCONTROL_H

/***********************************************************

TrajectoryControl.h
	Library containing the controller for flying on a commanded path with acceleration commands as input.
	
	
	Parameters to tune the controller are listed below and are meant for altering.
	
	struct SteeringSignals:
		Structure, containing all signals for steering the airplane.
		
	enum trajPIDs:
		Helps to access each PID controller innerwith the TrajectoryController-class.
		
	class TrajectoryController:
		Constructor: 
			Constructs the controller without the parameters listed below. Those can be set afterwards with the function setupTrCTRL().
			hal: Reference to the console for accessing its function (printf,...)
			dt: update interval: time period [milliseconds] (integer) at which the PID is accessed and updated.
			coFrequencyLPF: optional, standard=1MHz. Set the cut off frequency of the low-pass-filter for the derivatives of all PIDs.
			
		setPID(): 
			Set all gains, integral limit, and refresh interval for a specific PID.
		
		setPhiRef(): 
			Set the angle (0...360 degrees) for the reference vector phi_ref for the aileron control. Roll of the plane is controlled to this vector. If set to 0°, then the plane is controlled upright in body frame.
		
		update(): 
			Routine for updating all controller outputs of the trajectory control.
			time: For printing messages to console in one second cycles (for testing purposes).
			errorThrottle: Input of the throttle error for adjusting the throttle.
			aCMDb: Acceleration in body frame which the plane shall perform.
			gCMDb: Gravity term in body frame.
			stateVars: State variables for information about the state of the airplane.
			PhiRef: Input for angle of vector phi, will be changed and stored as in member-function setPhiRef().
			
		getPIDaccess():
			Possibility to get direct access to the specific PIDs of the trajectory controller. Possibilities of this access -> see PID.h
			
	setupTrCTRL():
		Function, that copies all parameters listed below to the trajectory controller.

************************************************************/

// ----------------------------------------------------------------------------------------
// Variables for tuning the Trajectory Controller

#define CO_Freq_LPF 1e6	// Cut-off frequency [Hz] for the low-pass-filter of the derivatives

#define errorThrottlePreGain 1.0/2.0 // Factor for the error signal going the throttle PID

// PID parameters for Throttle (for specific information, see PID.h.
#define Kp_Throttle 0.015	// Proportional gain
#define Ki_Throttle 0.0		// Integral gain
#define Kd_Throttle 0.04	// Derivative gain
#define IntLim_Throttle 1	// Integral limit
#define Casc_Throttle 1		// Factor to set a multiple of the update time the controller is updated

#define Kp_Aileron 0.9		// PID parameters for Aileron
#define Ki_Aileron 0.0
#define Kd_Aileron 0.2
#define IntLim_Aileron 0.5
#define Casc_Aileron 1

#define Kp_Rudder 0.1		// PID parameters for Rudder
#define Ki_Rudder 0
#define Kd_Rudder 0
#define IntLim_Rudder 1
#define Casc_Rudder 1

#define Kp_Elevator 0.1		// PID parameters for Elevator
#define Ki_Elevator 0.05
#define Kd_Elevator 0.01
#define IntLim_Elevator 1
#define Casc_Elevator 1

// ----------------------------------------------------------------------------------------



// Struct for all steering signals, handed to the airplane
struct SteeringSignals {
	float throttle, aileron, rudder, elevator;
};


// enum class for indicating PIDs in an array
enum trajPIDs {Throttle, Aileron, Rudder, Elevator};

//	Controller for the flight on trajectories and with aerobatics
//	Initialising needs the constructor and for every PID the 'set'-member-function!
class TrajectoryController {

public:
        // Constructor
        TrajectoryController (const AP_HAL::HAL& hal, const uint32_t dt, const float coFrequencyLPF = CO_Freq_LPF);

        // Destructor
        ~TrajectoryController();

        // setting the gains,integralLimit, and refreshInterval
        void setPID (const enum trajPIDs pidName, const float Kp, const float Ki, const float Kd, const float integralLimit = 100, const int8_t refreshInterval = 1);

	// edit PID
	void editPID(const enum trajPIDs pidName, float Kp, float Ki,float Kd);
	
        // Setting the reference vector phi_ref
        void setPhiRef (const float PhiRef);

        // Update Routine
        struct SteeringSignals update (uint32_t time, float errorThrottle, struct vector aCMDb, struct vector gCMDb, const struct StateVariables stateVars, const float PhiRef = 0);

        // Access to the PIDs
        // const is for denying any change in the PIDs
        PIDcontroller *getPIDAccess (const enum trajPIDs pidName);

private:
        const AP_HAL::HAL& m_rHAL;	// reference to the console (for printing messages)
        PIDcontroller* m_cPIDs[4];	// Array of the PIDs
        uint32_t m_iDt;				// Intervall time for update rate (for case refreshInterval==1); time in microseconds
        float m_fCOFrequencyLPF;	// Cut-off-frequency for the Low-Pass-Filter
        int8_t m_iRefreshIntervall;	// Factor of the intervall time, the controller is updated
        float m_fPhiRef;			// Reference Vector for Aileron

};


// Function to setup the TrajectoryController with the values from the defines from above.
void setupTrCTRL (TrajectoryController& trCTRL);

#include "TrajectoryControl.c"

#endif
