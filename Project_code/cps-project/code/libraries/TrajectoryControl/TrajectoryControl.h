#ifndef TRAJECTORYCONTROL_H
#define TRAJECTORYCONTROL_H

#include <math.h> //mathematical operator such as sqrt

/*##########################################################################################
                                   General Math Operations  
  ########################################################################################## */
// Cross Product
struct vector CrossProduct (const struct vector x, const struct vector y) {
	
	struct vector z;
	
	z.x = x.y*y.z - x.z*y.y;
	z.y = x.z*y.x - x.x*y.z;
	z.z = x.x*y.y - x.y*y.x;
	
	return(z);
}

// Scalar Product
float ScalarProduct (const struct vector x, const struct vector y) {
	
	float z;
	
	z = x.x * y.x + x.y * y.y + x.z * y.z;
	
	return(z);
}

//Euclidian norm for 3D Vector
float NormVector(const struct vector v) {
	float norm;
	norm=sqrt(v.x*v.x+v.y*v.y+v.z*v.z);
	return(norm);
}


/*##########################################################################################
                                   Class Trajectory Controller 
  ########################################################################################## */

// Struct for all steering signals, handed to the airplane
struct SteeringSignals {
	float throttle, aileron, rudder, elevator;
};


// enum class for indicating PIDs in an array
enum PID {Throttle, Aileron, Rudder, Elevator};

//	Controller for the flight on trajectories and with aerobatics
//	Initialising needs the constructor and for every PID the 'set'-member-function!
class TrajectoryController {

	public:
	
		// Constructor
		TrajectoryController (const AP_HAL::HAL& hal, const uint32_t dt, const float coFrequencyLPF = 0) :
			m_rHAL(hal), m_iDt(dt) , m_fCOFrequencyLPF(coFrequencyLPF) {

			// Initialising member variables
			int i;
			for(i=0;i<4;i++){
				m_cPIDs[i] = NULL;
			}
			
			// Initialising the PIDs
			for(i=0;i<4;i++){
				m_cPIDs[i] = new PIDcontroller(0,0,0,m_iDt);
				m_cPIDs[i]->setLPFfrequency(m_fCOFrequencyLPF);
			}
		}
		
		
		
		// Destructor
		~TrajectoryController () {
		
			// free all PIDs
			int i=0;
			for(i=0;i<4;i++){
				delete m_cPIDs[i];
				m_cPIDs[i] = NULL;
			}	
		}
		
		
		
		// setting the gains,integralLimit, and UpdateIntervall
		void setPID (const enum PID pidName, const float Kp, const float Ki, const float Kd, const float integralLimit = 100, const int8_t updateIntervall = 1) {
		
			int i;
			switch(pidName) {
				case Throttle :
					i = 0;
					break;
				case Aileron :
					i = 1;
					break;
				case Rudder :
					i = 2;
					break;
				case Elevator :
					i = 3;
					break;
				default:
					// Error-Routine
					return;
			}
			m_cPIDs[i]->setIntegralLimit(integralLimit);
			m_cPIDs[i]->setLPFfrequency(m_fCOFrequencyLPF);
			m_cPIDs[i]->setUpdateIntervall(updateIntervall);
			m_cPIDs[i]->setControllerGains(Kp,Ki,Kd);
			return;
		}
		

			
		// Setting the reference vector phi_ref
		void setPhiRef (const float PhiRef) {
			m_fPhiRef = M_PI / 180.0 * PhiRef;
			return;
		}
			
			
		// Update Routine
		struct SteeringSignals update (float DeltaL, struct vector aCMDb, struct vector gCMDb, const float PhiRef = 0, const int8_t aerobatOn = 0, struct vector eulerDesired = {0,0,0}) { 
			
			struct SteeringSignals t_cOut;
			struct vector t_vaCMDbYZ, t_veRoll, aCMDbSubG, t_vAileronCalc;
			float t_fAileronPID, t_fRudderPID, t_fElevatorPID;
			
			// Processing input variables
			m_fPhiRef = M_PI / 180.0 * PhiRef;
			
			// switch the mode of trajectories: Normal or aerobatic mode (Euler Angles)
			if (~aerobatOn) {
			// Normal mode
			
				// Throttle
				// Hand error of Throttle-PID to the controller and pass signal to output struct.
				t_cOut.throttle = m_cPIDs[Throttle]->update(DeltaL);
				
				// Aileron
				// Acceleration in y-z-Plane
				t_vaCMDbYZ.x = 0;
				t_vaCMDbYZ.y = aCMDb.y;
				t_vaCMDbYZ.z = aCMDb.z;
				// Find suitable eRoll: Calculate eRoll,1
				t_veRoll.x = 0;
				t_veRoll.y = - sin(m_fPhiRef);
				t_veRoll.z = - cos(m_fPhiRef);
				// Decide, whether eRoll,1 or eRoll,2 = -eRoll,1
				if (ScalarProduct(t_veRoll,t_vaCMDbYZ) < 0) {
					t_veRoll.y = -t_veRoll.y;
					t_veRoll.z = -t_veRoll.z;
				}
				// Build Cross Product and take x-component
				t_vAileronCalc = CrossProduct(t_veRoll,t_vaCMDbYZ);
				t_fAileronPID = t_vAileronCalc.x;
				// Take the arcsin of the x-component of the cross product
				t_fAileronPID = asin(t_fAileronPID);
				// Give the error of the Aileron to the Aileron-PID
				t_cOut.aileron = m_cPIDs[Aileron]->update(t_fAileronPID);
				
				// Rudder
				// Subtract gravity from acceleration
				aCMDbSubG.x = aCMDb.x - gCMDb.x;
				aCMDbSubG.y = aCMDb.y - gCMDb.y;
				aCMDbSubG.z = aCMDb.z - gCMDb.z;
				// Choose the y-Component
				t_fRudderPID = aCMDbSubG.y;
				// Give the error of the Rudder to the Rudder-PID
				t_cOut.rudder = m_cPIDs[Rudder]->update(t_fRudderPID);
				
				// Elevator
				// Choose the z-component of the acceleration subtracted by gravity
				t_fElevatorPID = aCMDbSubG.z;
				// Give the error of the Elevator to the Elevator-PID
				t_cOut.elevator = m_cPIDs[Elevator]->update(t_fElevatorPID);
				
			}
			// Aerobatic control mode (Euler Angles)
			else {
			
			}
			
			return(t_cOut);
			
		}

		
		
		// Access to the PIDs
		// const is for denying any change in the PIDs
		const PIDcontroller *getPIDAccess (const enum PID pidName) {
			
			int i;
			switch(pidName) {
				case Throttle :
					i = 0;
					break;
				case Aileron :
					i = 1;
					break;
				case Rudder :
					i = 2;
					break;
				case Elevator :
					i = 3;
					break;
				default:
					// Error-Routine
					return(NULL);
			}
			
			return(m_cPIDs[i]);}
		
		
		
	private:
		const AP_HAL::HAL& m_rHAL;	// reference to the console (for printing messages)
		PIDcontroller* m_cPIDs[4];	// Array of the PIDs
		uint32_t m_iDt;	// Intervall time for update rate (for case updateIntervall==1); time in microseconds
		float m_fCOFrequencyLPF;	// Cut-off-frequency for the Low-Pass-Filter
		int8_t m_iUpdateIntervall;	// Factor of the intervall time, the controller is updated
		float m_fPhiRef;	// Reference Vector

};



// Variables for the Trajectory Controller
#define CO_Freq_LPF 0	// Cut-off frequency for the low-pass-filter of the derivatives

#define Kp_Throttle 0.2
#define Ki_Throttle 0
#define Kd_Throttle 0
#define IntLim_Throttle 100
#define Casc_Throttle 1

#define Kp_Aileron 0.2
#define Ki_Aileron 0
#define Kd_Aileron 0
#define IntLim_Aileron 100
#define Casc_Aileron 1

#define Kp_Rudder 0.2
#define Ki_Rudder 0
#define Kd_Rudder 0
#define IntLim_Rudder 100
#define Casc_Rudder 1

#define Kp_Elevator 0.2
#define Ki_Elevator 0
#define Kd_Elevator 0
#define IntLim_Elevator 100
#define Casc_Elevator 1

void setupTrCTRL (TrajectoryController& trCTRL) {

	trCTRL.setPID ( Throttle, Kp_Throttle, Ki_Throttle, Kd_Throttle, IntLim_Throttle, Casc_Throttle );
	trCTRL.setPID ( Aileron, Kp_Aileron, Ki_Aileron, Kd_Aileron, IntLim_Aileron, Casc_Aileron );
	trCTRL.setPID ( Rudder, Kp_Rudder, Ki_Rudder, Kd_Rudder, IntLim_Rudder, Casc_Rudder );
	trCTRL.setPID ( Elevator, Kp_Elevator, Ki_Elevator, Kd_Elevator, IntLim_Elevator, Casc_Elevator );

	return;
}



#endif
