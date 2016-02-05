#ifndef FORMULASFORSTATEVARIABLES_H
#define FORMULASFORSTATEVARIABLES_H


// Declaration of a 3-dimensional vector
class vector {

public:
    float x,y,z;

    // Copy Function
    void importVector (vector refVector);
};


// Formulas for calculating derivatives of the state variables
struct vector derivativeAngularRate( struct vector pqr, struct vector phiThetaPsi );
struct vector derivativeVelocity( struct vector uvw, struct vector phiThetaPsi );


// Formulas for calculating the vehicle-velocity plus the vehicle-acceleration
float velocity( struct vector uvw );
float acceleration ( struct vector uvw, struct vector uvwDot );


// Transformation from NED-frame to Body-frame
struct vector NEDtoBODY ( const struct vector ned, const struct vector phiThetaPsi );

// Transformation from Body-frame to NED-frame
// STILL HAVE TO VERIFY IF INVERSE OF BODYTONED IS JUST THE TRANSPOSE
struct vector BODYtoNED ( const struct vector body, const struct vector phiThetaPsi );


// Container for all state Variables of the plane with copy function.
class StateVariables {

public:
	struct vector uvw, uvwDot ,pqr, phiThetaPsi, phiThetaPsiDot, pnPePd, pnPePdDot;
	float groundSpeed, groundSpeedDot;
        struct vector pnPePdDotDot;

        // Copies the state variables from the given class reference
        void importData (StateVariables refStateVars);
};

// From the measured data of the plane, calculate all necessary state variables.
struct StateVariables calculateStateVariables (const struct sample dataSample);



#include "StateVariables.c"
#endif
