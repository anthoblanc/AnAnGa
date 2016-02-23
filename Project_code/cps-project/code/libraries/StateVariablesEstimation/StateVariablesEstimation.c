
// Checking the flight position, if roll or pitch above lossAngle, the GPS-signal can not be tracked anymore
    // Loss Angle has to be between 0...180 Degrees
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


// Estimating the state variables, when GPS-connection is lost
void estimateStateVars (uint32_t time, const AP_HAL::HAL& hal,StateVariables& newVars, StateVariables oldVars){

    newVars.phiThetaPsiDot = derivativeAngularRate (newVars.pqr,oldVars.phiThetaPsi);

    newVars.phiThetaPsi = addVector( multiplyScalarToVector( addVector(newVars.phiThetaPsiDot,oldVars.phiThetaPsiDot), 0.5*PERIOD/1e6 ), oldVars.phiThetaPsi );
	
    if (newVars.phiThetaPsi.x>M_PI){newVars.phiThetaPsi.x -= 2*M_PI;}
    if (newVars.phiThetaPsi.x<-M_PI){newVars.phiThetaPsi.x += 2*M_PI;}
    if (newVars.phiThetaPsi.y>M_PI){newVars.phiThetaPsi.y -= 2*M_PI;}
    if (newVars.phiThetaPsi.y<-M_PI){newVars.phiThetaPsi.y += 2*M_PI;}
    if (newVars.phiThetaPsi.z>M_PI){newVars.phiThetaPsi.z -= 2*M_PI;}
    if (newVars.phiThetaPsi.z<-M_PI){newVars.phiThetaPsi.z += 2*M_PI;}

	// Try: rCM to (1,1,1) and BODY to NED over all summands.
    newVars.pnPePdDotDot = addVector (BODYtoNED(newVars.accelerationBodyFrame,newVars.phiThetaPsi), radiusCoefficientMatrix(CrossProduct(newVars.pqr,oldVars.uvw)) );
vector x = CrossProduct(newVars.pqr,oldVars.uvw);

    newVars.pnPePdDot = addVector( multiplyScalarToVector( addVector(newVars.pnPePdDotDot,oldVars.pnPePdDotDot), 0.5*PERIOD/1e6 ), oldVars.pnPePdDot );

    newVars.uvw = NEDtoBODY(newVars.pnPePdDot,newVars.phiThetaPsi);
	
// Edit: Add a factor of about 3*(3.8?) to the pnPePdDot
    newVars.pnPePd = addVector( multiplyScalarToVector( addVector(newVars.pnPePdDot,oldVars.pnPePdDot), 0.5*PERIOD/1e6 ), oldVars.pnPePd );

    if(time>=nextPrint){
        //hal.console->printf("%f\t%f\t%f\t",newVars.phiThetaPsiDot.x,newVars.phiThetaPsiDot.y,newVars.phiThetaPsiDot.z);
        //hal.console->printf("%f\t%f\t%f\t",newVars.phiThetaPsi.x,newVars.phiThetaPsi.y,newVars.phiThetaPsi.z);
        //hal.console->printf("%f\t%f\t%f\t",newVars.pnPePdDotDot.x,newVars.pnPePdDotDot.y,newVars.pnPePdDotDot.z);
        //hal.console->printf("%f\t%f\t%f\t",x.x,x.y,x.z);
        //hal.console->printf("%f\t%f\t%f\t",newVars.pnPePdDot.x,newVars.pnPePdDot.y,newVars.pnPePdDot.z);
        //hal.console->printf("%f\t%f\t%f\t",newVars.uvw.x,newVars.uvw.y,newVars.uvw.z);
        //hal.console->printf("%f\t%f\t%f\t",newVars.pnPePd.x,newVars.pnPePd.y,newVars.pnPePd.z);
    }
}
