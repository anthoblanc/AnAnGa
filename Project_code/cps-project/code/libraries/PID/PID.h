#ifndef PID_H
#define PID_H

//using namespace std;	// Delete, when in Linux, activate, when in Windows

float rollPIDOut = 0;
float pitchPIDOut = 0;
float speedPIDOut = 0;


class PIDcontroller {

public:
        PIDcontroller (const float Kp, const float Ki, const float Kd, const uint32_t dt,
                              const int8_t updateIntervall = 1);



        // Setting the Integral Limit
        /*	Sets the absolute Value of the input value as the integral limit. Value should be the maximal output value.
        */
        void setIntegralLimit (const float integralLimit);



        // Setting the Low-Pass-Filter cut-off-frequency
        void setLPFfrequency (const float RC);



        // Setting the update intervall
        void setUpdateIntervall (const int8_t updateIntervall);



        // Adjusting the Controller Gains Proportional, Integral, Derivative
        void setControllerGains (const float Kp, const float Ki, const float Kd);



        // Update Routine.
        /* 	If derivative not available, leave out this variable so not a number will be chosen	and quasi-derivative is then calculated.
        */
        float update (float error, const float derivative = NAN);


        // Reading the Update Success rate
        /*	For analysis of the quality of the input signal
        */
        float updateSuccessRate();

private:
        float m_fPrevOutput;		// Value of the last calculated Output Value
        float m_fPrevError[2];		// History of Error-Input-Values; [0]: oldest
        float m_fPrevUpdateError;	// Error-Value of the last update (for derivative)
        float m_fPrevDerivative;	// Value of the last derivative (for Low-Pass-Filter)
        float m_fAlpha;				// Factor for LP-Filter: Indication of Filtering Treshold
        float m_fIntegral;			// Actual Integral-Value
        float m_fIntegralLimit;		// Boundary of the Integral
        float m_fKp, m_fKi, m_fKd;	// Gain factors of Porportional, integral, and derivative Part
        int8_t m_iUpdateIntervall;	// Factor of the intervall time, the controller is updated
        int8_t m_iUpdateIntervallCounter;	// Counter for the next execution of the controller
        uint32_t m_iDt;	// Intervall time for update rate (for case updateIntervall==1); time in microseconds
        uint32_t m_iUpdateCounter;	// Counter of sum of updated input error values
        uint32_t m_iBadInputCounter;	// Counting number of bad input error values


};


#include "PID.c"

#endif
