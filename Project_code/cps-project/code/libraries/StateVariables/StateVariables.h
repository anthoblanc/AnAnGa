#ifndef FORMULASFORSTATEVARIABLES_H
#define FORMULASFORSTATEVARIABLES_H


// Declaration of a 3-dimensional vector
struct vector {
    float x,y,z;
};


// Formulas for calculating derivatives of the state variables
struct vector derivativeAngularRate( struct vector pqr, struct vector phiThetaPsi );
struct vector derivativeVelocity( struct vector uvw, struct vector phiThetaPsi );


// Formulas for calculating the vehicle-velocity plus the vehicle-acceleration
float velocity( struct vector uvw );
float acceleration ( struct vector uvw, struct vector uvwDot );


// Transformation from NED-frame to Body-frame
struct vector NEDtoBODY ( const struct vector ned, const struct vector phiThetaPsi );


// From the measured data of the plane, calculate all necessary state variables.
struct StateVariables {
	struct vector uvw, uvwDot ,pqr, phiThetaPsi, phiThetaPsiDot, pnPePd, pnPePdDot;
	float groundSpeed, groundSpeedDot;
};

struct StateVariables calculateStateVariables (const struct sample dataSample);



#include "StateVariables.c"
#endif
