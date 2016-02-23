

// Constructor
TrajectoryController::TrajectoryController (const AP_HAL::HAL& hal, const uint32_t dt, const float coFrequencyLPF) :
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
TrajectoryController::~TrajectoryController () {

        // free all PIDs
        int i=0;
        for(i=0;i<4;i++){
                delete m_cPIDs[i];
                m_cPIDs[i] = NULL;
        }
}



// setting the gains,integralLimit, and refreshInterval
void TrajectoryController::setPID (const enum trajPIDs pidName, const float Kp, const float Ki, const float Kd, const float integralLimit, const int8_t refreshInterval) {

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
        m_cPIDs[i]->setRefreshInterval(refreshInterval);
        m_cPIDs[i]->setControllerGains(Kp,Ki,Kd);
        return;
}



// Setting the reference vector phi_ref
void TrajectoryController::setPhiRef (const float PhiRef) {
        m_fPhiRef = M_PI / 180.0 * PhiRef;
        return;
}


// Update Routine
struct SteeringSignals TrajectoryController::update (uint32_t time, float errorThrottle, struct vector aCMDb, struct vector gCMDb, const struct StateVariables stateVars, const float PhiRef) {

        struct SteeringSignals t_cOut;
        struct vector t_vaCMDbYZ, t_veRoll, aCMDbSubG, t_vAileronCalc;
        float t_fAileronPID, t_fRudderPID, t_fElevatorPID;

        // Processing input variables
        m_fPhiRef = M_PI / 180.0 * PhiRef;

        // Throttle
        // Hand error of Throttle-PID to the controller and pass signal to output struct.
        errorThrottle *= errorThrottlePreGain;
        errorThrottle += stateVars.groundSpeed;
        t_cOut.throttle = m_cPIDs[Throttle]->update(errorThrottle);

        // Aileron
        // Acceleration in y-z-Plane
        t_vaCMDbYZ.x = 0;
        t_vaCMDbYZ.y = -aCMDb.y;
        t_vaCMDbYZ.z = -aCMDb.z;
        // Find suitable eRoll: Calculate eRoll,1
        t_veRoll.x = 0;
        t_veRoll.y = sin(m_fPhiRef);
        t_veRoll.z = cos(m_fPhiRef);
        // Decide, whether eRoll,1 or eRoll,2 = -eRoll,1 <------- Deactivated, not good for Roll maneuver and else not necessary.
       /* if (ScalarProduct(t_veRoll,t_vaCMDbYZ) < 0) {
                t_veRoll.y = -t_veRoll.y;
                t_veRoll.z = -t_veRoll.z;
        }*/ 
	   // Build Cross Product and take x-component
        float factor = 1.0/NormVector(t_vaCMDbYZ);
        t_vAileronCalc = CrossProduct(t_veRoll,multiplyScalarToVector(t_vaCMDbYZ,factor));
        t_fAileronPID = t_vAileronCalc.x;
        // Take the arcsin of the x-component of the cross product
        t_fAileronPID = constrain(t_fAileronPID,-1,1);
        t_fAileronPID = asin(t_fAileronPID);

        // Give the error of the Aileron to the Aileron-PID
        t_cOut.aileron = m_cPIDs[Aileron]->update(t_fAileronPID);

        // Rudder
        // Subtract gravity from acceleration
        aCMDbSubG.x = aCMDb.x + gCMDb.x;
        aCMDbSubG.y = aCMDb.y + gCMDb.y;
        aCMDbSubG.z = aCMDb.z + gCMDb.z;
        // Choose the y-Component
        t_fRudderPID = -aCMDbSubG.y;
        // Give the error of the Rudder to the Rudder-PID
        t_cOut.rudder = m_cPIDs[Rudder]->update(t_fRudderPID);

        // Elevator
        // Choose the z-component of the acceleration subtracted by gravity
        t_fElevatorPID = -aCMDbSubG.z;
        // Give the error of the Elevator to the Elevator-PID
        t_cOut.elevator = m_cPIDs[Elevator]->update(t_fElevatorPID);

        if (time >= nextPrint){
            //hal.console->printf("vector: (%f,%f), ref: (%f,%f), error: %f\n\n",t_vaCMDbYZ.y,t_vaCMDbYZ.z,t_veRoll.y,t_veRoll.z,t_fAileronPID);
            //hal.console->printf("factor, Aileron: %f,%f\t",factor,t_fAileronPID);
        }

        return(t_cOut);
}



// Access to the PIDs
// const is for denying any change in the PIDs
const PIDcontroller *TrajectoryController::getPIDAccess (const enum trajPIDs pidName) {

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

        return(m_cPIDs[i]);	// Adress operator & ???
}



// Function to setup the TrajectoryController with the values from the defines from above.
void setupTrCTRL (TrajectoryController& trCTRL) {

        trCTRL.setPID ( Throttle, Kp_Throttle, Ki_Throttle, Kd_Throttle, IntLim_Throttle, Casc_Throttle );
        trCTRL.setPID ( Aileron, Kp_Aileron, Ki_Aileron, Kd_Aileron, IntLim_Aileron, Casc_Aileron );
        trCTRL.setPID ( Rudder, Kp_Rudder, Ki_Rudder, Kd_Rudder, IntLim_Rudder, Casc_Rudder );
        trCTRL.setPID ( Elevator, Kp_Elevator, Ki_Elevator, Kd_Elevator, IntLim_Elevator, Casc_Elevator );

        return;
}
