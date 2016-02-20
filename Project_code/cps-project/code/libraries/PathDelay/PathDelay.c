
// Class PathDelay
// Constructor
PathDelay::PathDelay(){

    prevDesiredPath = {-365,-400,0.87};

}

// Update routine
float PathDelay::update(vector desiredPath, vector deltaLength){

    float distanceBehind;
    vector pathVelocity;

	// Build the velocity vector of the desired path
    pathVelocity = subtractVector(desiredPath,prevDesiredPath); 
	
	// Calculate the distance concerning the path velocity the airplane lags behind.
    distanceBehind = ScalarProduct(pathVelocity,deltaLength) / NormVector(pathVelocity);

	// Copy the current position to the previous position for the next update in the following time step.
    prevDesiredPath.importVector(desiredPath);

    return(distanceBehind);
}



