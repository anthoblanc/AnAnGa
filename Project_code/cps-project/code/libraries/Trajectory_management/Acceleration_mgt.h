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



#include "Acceleration_mgt.cpp"

#endif
