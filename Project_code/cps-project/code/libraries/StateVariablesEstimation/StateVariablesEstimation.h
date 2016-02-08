#ifndef SVESTIMATION_H
#define SVESTIMATION_H


// Checking the flight position, if roll or pitch above lossAngle, the GPS-signal can not be tracked anymore
    // Loss Angle has to be between 0...90(...180) Degrees
int NoSignalAvailableGPS (vector Euler, float lossAngle_Degrees);


// Estimating the state variables, when GPS-connection is lost
void estimateStateVars (uint32_t hardware_time,const AP_HAL::HAL& hal,StateVariables& newVars, StateVariables oldVars);


#include "StateVariablesEstimation.c"
#endif
