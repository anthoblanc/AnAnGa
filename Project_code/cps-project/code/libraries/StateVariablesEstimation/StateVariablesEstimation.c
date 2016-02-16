
int NoSignalAvailableGPS (vector Euler, float lossAngle_Degrees){

    int noSignal = 1;
    float angle = lossAngle_Degrees * M_PI / 180;

    // Checking for Pitch
    if ( (Euler.y>=-angle && Euler.y<=angle) || (Euler.y>=360-angle && Euler.y<=360) ) {
        if ( (Euler.x>=-angle && Euler.x<=angle) || (Euler.x>=360-angle && Euler.x<=360) ) {
            noSignal = 0;
        }
    }

    return(noSignal);
}

void estimateStateVars (uint32_t hardware_time,const AP_HAL::HAL& hal,StateVariables& newVars, StateVariables oldVars){

    newVars.phiThetaPsiDot = derivativeAngularRate (newVars.pqr,oldVars.phiThetaPsi);

    newVars.phiThetaPsi = addVector( multiplyScalarToVector( addVector(newVars.phiThetaPsiDot,oldVars.phiThetaPsiDot), 0.5*PERIOD/1e6 ), oldVars.phiThetaPsi );

    newVars.pnPePdDotDot = addVector( BODYtoNED(newVars.accelerationBodyFrame,newVars.phiThetaPsi), radiusCoefficientMatrix(CrossProduct(newVars.pqr,oldVars.uvw)) );

    newVars.pnPePdDot = addVector( multiplyScalarToVector( addVector(newVars.pnPePdDotDot,oldVars.pnPePdDotDot), 0.5*PERIOD/1e6 ), oldVars.pnPePdDot );

    newVars.uvw = NEDtoBODY(newVars.pnPePdDot,newVars.phiThetaPsi);

    newVars.pnPePd = addVector( multiplyScalarToVector( addVector(newVars.pnPePdDot,oldVars.pnPePdDot), 0.5*PERIOD/1e6 ), oldVars.pnPePd );

    if(hardware_time>=nextPrint){
    //hal.console->printf("est: (%f,%f,%f)\n",newVars.pnPePd.x,newVars.pnPePd.y,newVars.pnPePd.z);
    }
}
