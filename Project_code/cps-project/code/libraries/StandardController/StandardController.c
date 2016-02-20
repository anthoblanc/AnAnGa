
// Class StandardController
// Constructor
StandardController::StandardController( const AP_HAL::HAL& hal ) : m_rHAL(hal) {

        PID.Heading.setIntegralLimit (1);
        PID.Heading.setLPFfrequency(0);
        PID.Roll.setIntegralLimit (1);
        PID.Roll.setLPFfrequency(0);
        PID.Altitude.setIntegralLimit (10);
        PID.Altitude.setLPFfrequency(10);
        PID.ClimbRate.setIntegralLimit (1);
        PID.ClimbRate.setLPFfrequency(30);
        PID.Pitch.setIntegralLimit (1);
        PID.Pitch.setLPFfrequency(0);
        PID.Speed.setIntegralLimit (1);
        PID.Speed.setLPFfrequency(0);
}

// Instructions for the setup routine
void StandardController::setup(/* Possibility here, to override the control targets or hard bounds */) {
        // Initial flight control targets, note altitude is down.
        target.heading = TargetHeading*(M_PI/180.0);
        target.speed = TargetSpeed;
        target.altitude = TargetAltitude;

        // Compute error in heading, ensuring it is in the range -Pi to Pi
        if(target.heading>M_PI){target.heading = target.heading - 2*M_PI;}
        if(target.heading<-M_PI){target.heading = target.heading + 2*M_PI;}

        // Set maximum pitch and roll to 30 degrees.
        hardBound.maxPitch = MaxPitch*(M_PI/180.0);
        hardBound.maxRoll = MaxRoll*(M_PI/180.0);

        return;
}

// Update Routine in the Main-Function
struct SteeringSignals StandardController::update (const struct sample dataSample){

        struct SteeringSignals out;

        // From the measured data of the plane, calculate all necessary state variables.
        m_stateVars = calculateStateVariables (dataSample);

        // Compute heading PID
        float headingPIDOut = PID.Heading.update(target.heading-m_stateVars.phiThetaPsi.z, m_stateVars.phiThetaPsiDot.z);

        // Constrain output of heading PID such that it is a valid target roll
        headingPIDOut = constrain(headingPIDOut,-hardBound.maxRoll,hardBound.maxRoll);
        // Compute roll PID
        out.aileron = PID.Roll.update(headingPIDOut - m_stateVars.phiThetaPsi.x, m_stateVars.phiThetaPsiDot.x);

        // Compute altitude PID
        float altitudePIDOut = PID.Altitude.update(-target.altitude + m_stateVars.pnPePd.z, -m_stateVars.pnPePdDot.z);

        // Compute climb rate PID
        float climbRatePIDOut = PID.ClimbRate.update(altitudePIDOut + m_stateVars.pnPePdDot.z);

        // Constrain output of climb rate PID such that it is a valid target pitch
        climbRatePIDOut = constrain (climbRatePIDOut,-hardBound.maxPitch/4, +hardBound.maxPitch);

        // Compute pitch PID
        out.elevator = PID.Pitch.update(climbRatePIDOut - m_stateVars.phiThetaPsi.y, m_stateVars.phiThetaPsiDot.y);

        // Compute speed PID
        out.throttle = PID.Speed.update(target.speed - m_stateVars.groundSpeed,m_stateVars.groundSpeedDot);

        return(out);

}


// Getting values of the state variables of the plane
const struct StateVariables *StandardController::getStateVariables () {
        return(&m_stateVars);
}


// Access to the PIDs
// const is for denying any change in the PIDs
const PIDcontroller *StandardController::getPIDAccess (enum stdPIDs pidName) {

	switch(pidName) {
			case heading :
					return(&PID.Heading);
					break;
			case roll :
					return(&PID.Roll);
					break;
			case altitude :
					return(&PID.Altitude);
					break;
			case climbRate :
					return(&PID.ClimbRate);
					break;
			case pitch :
					return(&PID.Pitch);
					break;
			case speed :
					return(&PID.Speed);
					break;
			default:
					// Error-Routine
					return(NULL);
	}
	
}




