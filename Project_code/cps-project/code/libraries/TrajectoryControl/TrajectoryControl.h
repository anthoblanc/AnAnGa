#ifndef TRAJECTORYCONTROL_H
#define TRAJECTORYCONTROL_H

// General Math Operations
// Cross Product
struct vector CrossProduct (const struct vector x, const struct vector y){
	
	struct vector z;
	
	z.x = x.y*y.z - x.z*y.y;
	z.y = x.z*y.x - x.x*y.z;
	z.z = x.x*y.y - x.y*y.x;
	
	return(z);
}



// Struct for all steering signals, handed to the airplane
struct SteeringSignals {
	float throttle, aileron, rudder, elevator;
};



// enum class for indicating PIDs in an array
enum PID {Throttle, Aileron, Rudder, Elevator};



// Class Trajectory Controller
//	Controller for the flight on trajectories and with aerobatics
//	Initialising needs the constructor and for every PID the 'set'-member-function!

class TrajectoryController {

	public:
	
		// Constructor
		TrajectoryController (const uint32_t dt, const float coFrequencyLPF = 0) :
			m_iDt(dt) , m_fCOFrequencyLPF(coFrequencyLPF) {
		
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
			
			
			
		// Update Routine
		struct SteeringSignals update (float DeltaL, struct vector aCMDb, struct vector gCMDb, float PhiRef = 90, int8_t aerobatOn = 0, struct vector eulerDesired = {0,0,0}) { 
			
			// code here
			struct SteeringSignals out;
				
			return(out);
			
		}

		
		
		
	private:
		PIDcontroller* m_cPIDs[4];	// Array of the PIDs
		uint32_t m_iDt;	// Intervall time for update rate (for case updateIntervall==1); time in microseconds
		float m_fCOFrequencyLPF;	// Cut-off-frequency for the Low-Pass-Filter
		int8_t m_iUpdateIntervall;	// Factor of the intervall time, the controller is updated

};




#endif