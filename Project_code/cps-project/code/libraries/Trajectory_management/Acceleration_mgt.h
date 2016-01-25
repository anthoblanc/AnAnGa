#ifndef ACCELERATION_MGT_H
#define ACCELERATION_MGT_H


/*
#########################################################################""
			Get_Acc_straigth();
#########################################################################

This function provide the accelerator vector to reach a point with the shortest path.

%% CHOICE TO MAKE %%
%%%%%
NEED:
This function need to #include the 
struct StateVariable {
    struct vector uvw, uvwDot ,pqr, phiThetaPsi, phiThetaPsiDot, pnPePd, pnPePdDot;
    float groundSpeed, groundSpeedDot;
};
%%%%%
or
%%%%%
Input:

const struct vector velocity_refgnd
const struct vector trajectory_refgn
%%%%%

OUPUT:
acceleration vector expressed in the gnd referential

*/
struct vector Get_Acc_straigth(const AP_HAL::HAL& hal,struct vector velocity_refgnd, struct vector trajectory_refgnd);



/*
Provide_nice_curve_acc

Provide a nice curve to reach a point (ie an acceleration vector perpendicalaire to the shoter path)
It is basically a 1-space dimension problem: d(t) (time is the variable)

input:
-struc vector: origine point (referential: fixed point on earth)
-struc vector: goal point (same referential)
-struc vector: body frame (to know the direction of the plane)

output:
-struct vector: accelearation vector to have a nice approach curve

question: 
If I include FORMULASFORSTATEVARIABLES_H, I don't need to have the body frame as input?

Thing to keep in mind:
-add gravitation acceleration
-referential

Subtask to do:
-Calculate L
-Calculate V (we keep it constant! We only change the direction!)
-Express the transfer function
-Calculate the space of d (perpendicalare of the shortest path line and parrallele to the ground)
-Express a!
*/

struct vector Provide_nice_curve_acc();

#include "Acceleration_mgt.cpp"

#endif
