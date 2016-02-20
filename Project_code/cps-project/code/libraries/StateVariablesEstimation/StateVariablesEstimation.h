#ifndef SVESTIMATION_H
#define SVESTIMATION_H

/***********************************************************

StateVariablesEstimation.h
	Library containing a function to estimate the stateVariables for the case, the GPS signal is not available.
	
	NoSignalAvailableGPS():
		Examination, whether with the state of the airplane, a GPS-tracking is not possible.
		This is necessary, because signals of the simulator are ideal and have to be "deleted" manually.
		lossAngle_Degrees: Loss Angle has to be between 0...180 degrees
		
	estimateStateVars():
		Estimation routine for the state variables, when GPS-connection is lost.
		time: For printing messages to console in one second cycles (for testing purposes).
		hal: Reference to the console for accessing its function (printf,...)
		newVars: New state variables (recent measurement) (Only uvw and pqr will be taken, the rest will be overwritten).
		oldVars: State variables of the previous time step.
	

************************************************************/

// Checking the flight position, if roll or pitch above lossAngle, the GPS-signal can not be tracked anymore
    // Loss Angle has to be between 0...180 Degrees
int NoSignalAvailableGPS (vector Euler, float lossAngle_Degrees);


// Estimating the state variables, when GPS-connection is lost
void estimateStateVars (uint32_t time, const AP_HAL::HAL& hal, StateVariables& newVars, StateVariables oldVars);


#include "StateVariablesEstimation.c"

#endif
