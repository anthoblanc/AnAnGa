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
void StandardController::setup(struct ControlTargets& target, struct HardBounds& hardBound) {
        // Initial flight control targets, note altitude is down.
        target.heading = 100.0*(M_PI/180.0);
        target.speed = 15;
        target.altitude = -100;

        // Compute error in heading, ensuring it is in the range -Pi to Pi
        if(target.heading>M_PI){target.heading = target.heading - 2*M_PI;}
        if(target.heading<-M_PI){target.heading = target.heading + 2*M_PI;}

        // Set maximum pitch and roll to 30 degrees.
        hardBound.maxPitch = 30.0*(M_PI/180.0);
        hardBound.maxRoll = 30.0*(M_PI/180.0);

        return;
}

// Update Routine in the Main-Function
struct SteeringSignals StandardController::update (const struct sample dataSample, const struct ControlTargets target, const struct HardBounds hardBound){

        struct SteeringSignals out;
        struct StateVariables stateVars;

        // From the measured data of the plane, calculate all necessary state variables.
        stateVars = calculateStateVariables (dataSample);
        uvw = stateVars.uvw;
        uvwDot = stateVars.uvwDot;
        pqr = stateVars.pqr;
        phiThetaPsi = stateVars.phiThetaPsi;
        phiThetaPsiDot = stateVars.phiThetaPsiDot;
        pnPePd = stateVars.pnPePd;
        pnPePdDot = stateVars.pnPePdDot;
        groundSpeed = stateVars.groundSpeed;
        groundSpeedDot = stateVars.groundSpeedDot;

        // Compute heading PID
        float headingPIDOut = PID.Heading.update(target.heading-phiThetaPsi.z, phiThetaPsiDot.z);

        // Constrain output of heading PID such that it is a valid target roll
        headingPIDOut = constrain(headingPIDOut,-hardBound.maxRoll,hardBound.maxRoll);
        // Compute roll PID
        out.aileron = PID.Roll.update(headingPIDOut - phiThetaPsi.x, phiThetaPsiDot.x);

        // Compute altitude PID
        float altitudePIDOut = PID.Altitude.update(-target.altitude + pnPePd.z, -pnPePdDot.z);

        // Compute climb rate PID
        float climbRatePIDOut = PID.ClimbRate.update(altitudePIDOut + pnPePdDot.z,0);

        // Constrain output of climb rate PID such that it is a valid target pitch
        climbRatePIDOut = constrain (climbRatePIDOut,-hardBound.maxPitch/4, +hardBound.maxPitch);

        // Compute pitch PID
        out.elevator = PID.Pitch.update(climbRatePIDOut - phiThetaPsi.y, phiThetaPsiDot.y);

        // Compute speed PID
        out.throttle = PID.Speed.update(target.speed - groundSpeed,groundSpeedDot);
//hal.console->printf("%f\t%f\t%f\t%f\t%f\t%f\n",-target.altitude + pnPePd.z,altitudePIDOut,altitudePIDOut + pnPePdDot.z,climbRatePIDOut,climbRatePIDOut - phiThetaPsi.y,out.elevator);
        return(out);

}

// Getting values of the state variables of the plane
struct vector StandardController::getVector (const enum States x) {
        struct vector out;
        switch(x) {
                case UVW: 		out = uvw;break;
                case UVWdot: 	out = uvwDot;break;
                case PQR:		out = pqr;break;
                case PhiThetaPsi: out = phiThetaPsi;break;
                case PhiThetaPsiDot: out = phiThetaPsiDot;break;
                case PnPePd:	out = pnPePd;break;
                case PnPePdDot:	out = pnPePdDot;break;
                default:		out = {0,0,0};break;
        }
        return(out);
}
float StandardController::getGroundSpeed () {return(groundSpeed);}
float StandardController::getGroundSpeedDot () {return(groundSpeedDot);}

// Access to the PIDs
// const is for denying any change in the PIDs
const struct PIDs *StandardController::getPIDAccess () {return(&PID);}
