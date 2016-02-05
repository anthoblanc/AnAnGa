
int SignalCheckGPS (vector Euler, float lossAngle_Degrees){

    int signal = 0;
    float angle = lossAngle_Degrees * M_PI / 180;

    // Checking for Pitch
    if ( (Euler.y>=-angle && Euler.y<=angle) || (Euler.y>=360-angle && Euler.y<=360) ) {
        if ( (Euler.x>=-angle && Euler.x<=angle) || (Euler.x>=360-angle && Euler.x<=360) ) {
            signal = 1;
        }
    }

    return(signal);
}

void estimateStateVars (StateVariables& newVars, StateVariables oldVars){

    newVars.phiThetaPsiDot = derivativeAngularRate (newVars.pqr,oldVars.phiThetaPsi);

    newVars.phiThetaPsi = addVector( multiplyScalarToVector( addVector(newVars.phiThetaPsiDot,oldVars.phiThetaPsiDot), 0.5*PERIOD/1e6 ), oldVars.phiThetaPsi );

    newVars.pnPePdDotDot = addVector( addVector( BODYtoNED(newVars.accelerationBodyFrame,newVars.phiThetaPsi), GRAVITY_NED ), radiusCoefficientMatrix(CrossProduct(newVars.pqr,oldVars.uvw)) );

    newVars.pnPePdDot = addVector( multiplyScalarToVector( addVector(newVars.pnPePdDotDot,oldVars.pnPePdDotDot), 0.5*PERIOD/1e6 ), oldVars.pnPePdDot );

    newVars.uvw = NEDtoBODY(newVars.pnPePdDot,newVars.phiThetaPsi);

    newVars.pnPePd = addVector( multiplyScalarToVector( addVector(newVars.pnPePdDot,oldVars.pnPePdDot), 0.5*PERIOD/1e6 ), oldVars.pnPePd );

}
