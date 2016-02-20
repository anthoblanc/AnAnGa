#ifndef PATHDELAY_H
#define PATHDELAY_H

/***********************************************************

PathDelay.h
	Library containing a class to calculate the distance of the airplane, it lags behind the trajectory at a given time. Output is for further processing for the input error of the throttle.
	
	update():
		Perform the calculation of the distance behind the desired path.
		desiredPath: vector of the position at which the airplane should be at this specific time step.
		deltaLength: Difference vector of the position where the airplane should be and where the airplane actually is.
	

************************************************************/

class PathDelay {

public:

    PathDelay();

    float update(vector desiredPath, vector deltaLength);

private:

    struct vector prevDesiredPath; // vector of the position where it should have been at the time step before.

};


#include "PathDelay.c"

#endif
