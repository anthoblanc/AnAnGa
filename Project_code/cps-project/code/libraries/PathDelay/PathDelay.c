

PathDelay::PathDelay(){

    prevDesiredPath = {-365,-400,0.87};

}

float PathDelay::update(vector desiredPath, vector deltaLength){

    float distanceBehind;
    vector pathVelocity;

    pathVelocity = subtractVector(desiredPath,prevDesiredPath);
    distanceBehind = ScalarProduct(pathVelocity,deltaLength) / NormVector(pathVelocity);

    prevDesiredPath.importVector(desiredPath);

    return(distanceBehind);
}



