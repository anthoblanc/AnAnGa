#ifndef TRAJECTORYCONTROL_H
#define TRAJECTORYCONTROL_H




/*##########################################################################################
                                   Class Trajectory Controller 
  ########################################################################################## */

// Struct for all steering signals, handed to the airplane
struct SteeringSignals {
	float throttle, aileron, rudder, elevator;
};


// enum class for indicating PIDs in an array
enum PID {Throttle, Aileron, Rudder, Elevator};

//	Controller for the flight on trajectories and with aerobatics
//	Initialising needs the constructor and for every PID the 'set'-member-function!
class TrajectoryController {

public:
        // Constructor
        TrajectoryController (const AP_HAL::HAL& hal, const uint32_t dt, const float coFrequencyLPF = 0);

        // Destructor
        ~TrajectoryController();

        // setting the gains,integralLimit, and UpdateIntervall
        void setPID (const enum PID pidName, const float Kp, const float Ki, const float Kd, const float integralLimit = 100, const int8_t updateIntervall = 1);

        // Setting the reference vector phi_ref
        void setPhiRef (const float PhiRef);

        // Update Routine
        struct SteeringSignals update (uint32_t time, float errorThrottle, struct vector aCMDb, struct vector gCMDb, const struct StateVariables stateVars, const float PhiRef = 0);

        // Access to the PIDs
        // const is for denying any change in the PIDs
        const PIDcontroller *getPIDAccess (const enum PID pidName);

private:
        const AP_HAL::HAL& m_rHAL;	// reference to the console (for printing messages)
        PIDcontroller* m_cPIDs[4];	// Array of the PIDs
        uint32_t m_iDt;	// Intervall time for update rate (for case updateIntervall==1); time in microseconds
        float m_fCOFrequencyLPF;	// Cut-off-frequency for the Low-Pass-Filter
        int8_t m_iUpdateIntervall;	// Factor of the intervall time, the controller is updated
        float m_fPhiRef;	// Reference Vector

};



// Variables for the Trajectory Controller
#define CO_Freq_LPF 0	// Cut-off frequency for the low-pass-filter of the derivatives

#define errorThrottlePreGain 1.0/2.0

#define Kp_Throttle 0.015
#define Ki_Throttle 0.0
#define Kd_Throttle 0.04
#define IntLim_Throttle 1
#define Casc_Throttle 1

#define Kp_Aileron 0.09
#define Ki_Aileron 0.0
#define Kd_Aileron 0.04
#define IntLim_Aileron 0.1
#define Casc_Aileron 1

#define Kp_Rudder 0.1
#define Ki_Rudder 0
#define Kd_Rudder 0.02
#define IntLim_Rudder 1
#define Casc_Rudder 1

#define Kp_Elevator 0.1
#define Ki_Elevator 0.05
#define Kd_Elevator 0.01
#define IntLim_Elevator 1
#define Casc_Elevator 1

void setupTrCTRL (TrajectoryController& trCTRL);

#include "TrajectoryControl.c"

#endif
