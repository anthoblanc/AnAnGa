
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

    // 3. xDotDot? is this maybe uvwDot?
    oldVars.pnPePdDotDot = BODYtoNED(oldVars.uvwDot,oldVars.phiThetaPsi);
    newVars.pnPePdDotDot = BODYtoNED(newVars.uvwDot,newVars.phiThetaPsi);

    newVars.pnPePdDot = addVector( multiplyScalarToVector( addVector(newVars.pnPePdDotDot,oldVars.pnPePdDotDot), 0.5*PERIOD/1e6 ), oldVars.pnPePdDot );

    newVars.uvw = NEDtoBODY(newVars.pnPePdDot,newVars.phiThetaPsi);

    newVars.pnPePd = addVector( multiplyScalarToVector( addVector(newVars.pnPePdDot,oldVars.pnPePdDot), 0.5*PERIOD/1e6 ), oldVars.pnPePd );

}
