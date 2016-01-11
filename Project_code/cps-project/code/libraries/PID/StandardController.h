#ifndef STANDARDCONTROLLER_H
#define STANDARDCONTROLLER_H

// Set the gains for all PIDS
// Gains for pidHeading
#define Kp_Heading 1.5
#define Ki_Heading 0.0001
#define Kd_Heading 0.1
#define Casc_Heading 3

// Gains for pidRoll
#define Kp_Roll 1
#define Ki_Roll 0
#define Kd_Roll 0
#define Casc_Roll 1

// Gains for pidAltitude
#define Kp_Altitude 2
#define Ki_Altitude 0.01
#define Kd_Altitude 0.1
#define Casc_Altitude 3

// Gains for pidClimbRate
#define Kp_ClimbRate 1
#define Ki_ClimbRate 0
#define Kd_ClimbRate 0
#define Casc_ClimbRate 3

// Gains for pidPitch
#define Kp_Pitch 1
#define Ki_Pitch 0
#define Kd_Pitch 0
#define Casc_Pitch 1

// Gains for pidSpeed
#define Kp_Speed 1
#define Ki_Speed 0
#define Kd_Speed 0
#define Casc_Speed 1

// Declare PIDs
struct PIDs {
    PIDcontroller Heading;
    PIDcontroller Roll;
    PIDcontroller Altitude;
    PIDcontroller ClimbRate;
    PIDcontroller Pitch;
    PIDcontroller Speed;
};

enum StateVariables {UVW,UVWdot,PQR,PhiThetaPsi,PhiThetaPsiDot,PnPePd,PnPePdDot};
						
// Class Standardcontroller
class StandardController {

	public:
	
		// Constructor
		StandardController( const AP_HAL::HAL& hal ) : m_rHAL(hal) {

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
		void setup(struct ControlTargets& target, struct HardBounds& hardBound) {
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
		struct SteeringSignals update (const struct sample dataSample, const struct ControlTargets target, const struct HardBounds hardBound){
				
			struct SteeringSignals out;
			
			// From the measured data of the plane, calculate all necessary state variables.
			calculateStateVariables (dataSample, uvw, uvwDot, pqr, phiThetaPsi, pnPePd, phiThetaPsiDot, pnPePdDot, groundSpeed, groundSpeedDot);

			// Compute heading PID
			float headingPIDOut = PID.Heading.update(target.heading-phiThetaPsi.z, phiThetaPsiDot.z);

			// Constrain output of heading PID such that it is a valid target roll
			headingPIDOut = constrain(headingPIDOut,-hardBound.maxRoll,hardBound.maxRoll);
			// Compute roll PID
			out.aileron = PID.Roll.update(headingPIDOut - phiThetaPsi.x, phiThetaPsiDot.x);

			// Compute altitude PID
			float altitudePIDOut = PID.Altitude.update(-target.altitude + pnPePd.z, -pnPePdDot.z);

			// Compute climb rate PID
			float climbRatePIDOut = PID.ClimbRate.update(altitudePIDOut + pnPePdDot.z);

			// Constrain output of climb rate PID such that it is a valid target pitch
			climbRatePIDOut = constrain (climbRatePIDOut,-hardBound.maxPitch, +hardBound.maxPitch);

			// Compute pitch PID
			out.elevator = PID.Pitch.update(climbRatePIDOut - phiThetaPsi.y, phiThetaPsiDot.y);

			// Compute speed PID
			out.throttle = PID.Speed.update(target.speed - groundSpeed,groundSpeedDot);
    hal.console->printf("%f\t%f\t%f\t%f\t%f\t%f\n",-target.altitude + pnPePd.z,altitudePIDOut,altitudePIDOut + pnPePdDot.z,climbRatePIDOut,climbRatePIDOut - phiThetaPsi.y,out.elevator);			
			return(out);
		
		}
		
		// Getting values of the state variables of the plane
		struct vector getVector (const enum StateVariables x) {
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
		float getGroundSpeed () {return(groundSpeed);}
		float getGroundSpeedDot () {return(groundSpeedDot);}
		
		// Access to the PIDs
		// const is for denying any change in the PIDs
		const struct PIDs *getPIDAccess () {return(&PID);}
		
	private:
	
		const AP_HAL::HAL& m_rHAL;	// reference to the console (for printing messages)
		
		// All PIDs of the controller
		struct PIDs PID = { {Kp_Heading,Ki_Heading,Kd_Heading,PERIOD,Casc_Heading},
                        {Kp_Roll,Ki_Roll,Kd_Roll,PERIOD,Casc_Roll},
                        {Kp_Altitude,Ki_Altitude,Kd_Altitude,PERIOD,Casc_Altitude},
                        {Kp_ClimbRate,Ki_ClimbRate,Kd_ClimbRate,PERIOD,Casc_ClimbRate},
                        {Kp_Pitch,Ki_Pitch,Kd_Pitch,PERIOD,Casc_Pitch},
                        {Kp_Speed,Ki_Speed,Kd_Speed,PERIOD,Casc_Speed}};
		
		// Declaration of all state variables for controlling the vehicle
		struct vector uvw;
		struct vector uvwDot;
		struct vector pqr;
		struct vector phiThetaPsi;
		struct vector phiThetaPsiDot;
		struct vector pnPePd;
		struct vector pnPePdDot;
		float groundSpeed;
		float groundSpeedDot;

};

#endif
