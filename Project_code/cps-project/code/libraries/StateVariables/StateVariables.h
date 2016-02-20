#ifndef STATEVARIABLES_H
#define STATEVARIABLES_H


/***********************************************************

StateVariables.h
	Library containing a class to store, compute, and handle all relevant state varibles of an airplane.
	
	
	Class StateVariables:
		Storage of all position-, velocity-, and acceleration vectors plus the groundspeed-value and its derivative.
		importData(): A member-function to copy all values from one class to the other one.
		
	
	derivative AngularRate(): Calculates the derivatives of the Euler angles with the Euler angles and rotation rates pqr as input.
	
	derivativeVelocity(): Calculates the velocity in inertial frame with the Euler angles and the body velocity uvw.
	
	derivativeBodyVelocity(): Calculates the acceleration of the airplane in body frame with rotation rates pqr, body velocity uvw, and acceleration data.
	
	
	calculateStateVariables(): 
		Function assigns all variables, coming from the simulation, to the StateVariable-class and performs all necessary calculations to gather all values in StateVariables-class.

************************************************************/


// Container for all state Variables of the plane with copy function.
class StateVariables {

public:
	struct vector uvw, uvwDot ,pqr, phiThetaPsi, phiThetaPsiDot, pnPePd, pnPePdDot;
	float groundSpeed, groundSpeedDot;
        struct vector pnPePdDotDot, accelerationBodyFrame;

        // Copies the state variables from the given class reference
        void importData (StateVariables refStateVars);
};

// Formulas for calculating derivatives of the state variables
struct vector derivativeAngularRate( struct vector pqr, struct vector phiThetaPsi );
struct vector derivativeVelocity( struct vector uvw, struct vector phiThetaPsi );
struct vector derivativeBodyVelocity( struct vector pqr, struct vector uvw, struct vector acc);

// Formulas for calculating the vehicle-velocity plus the vehicle-acceleration
float velocity( struct vector uvw );
float acceleration ( struct vector uvw, struct vector uvwDot );



// From the measured data of the plane, calculate all necessary state variables.
struct StateVariables calculateStateVariables (const struct sample dataSample);



#include "StateVariables.c"
#endif
