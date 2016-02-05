#ifndef FORMULASFORSTATEVARIABLES_H
#define FORMULASFORSTATEVARIABLES_H





// Formulas for calculating derivatives of the state variables
struct vector derivativeAngularRate( struct vector pqr, struct vector phiThetaPsi );
struct vector derivativeVelocity( struct vector uvw, struct vector phiThetaPsi );


// Formulas for calculating the vehicle-velocity plus the vehicle-acceleration
float velocity( struct vector uvw );
float acceleration ( struct vector uvw, struct vector uvwDot );



// Container for all state Variables of the plane with copy function.
class StateVariables {

public:
	struct vector uvw, uvwDot ,pqr, phiThetaPsi, phiThetaPsiDot, pnPePd, pnPePdDot;
	float groundSpeed, groundSpeedDot;
        struct vector pnPePdDotDot, accelerationBodyFrame;

        // Copies the state variables from the given class reference
        void importData (StateVariables refStateVars);
};

// From the measured data of the plane, calculate all necessary state variables.
struct StateVariables calculateStateVariables (const struct sample dataSample);



#include "StateVariables.c"
#endif
