#ifndef PID_H
#define PID_H

/***********************************************************

PID.h
	Library for a digital Proportional-Integral-Derivative Contoller
	
	
	Adjustable update-rate, refresh-rate, integral limit, Low-Pass-Filter cutoff frequency for derivative filtering.
	Automatical switching from derivative out of historic data and a derivative signal, given with the arguments (if argument available).
	
        Constructor:
		Kp, Ki, Kd: Gains for the proportional, integral, and derivative component:	out = Kp * error + Ki * int(error)dt + Kd * d/dt(error)
		dt: update interval: time period [milliseconds] (integer) at which the PID is accessed and updated -> important for Derivative and Low-Pass-Filter
		refreshIntervall: (integer), standard=1. Multiple of the update interval at which the PID is updated -> Forcing the PID to a slower update rate (e.g. cascaded PIDs)
		
	setIntegralLimit():
                standard=100. Set a constraint for the integral of the error. Above or below this treshold, the value remains at the treshold.

        setLPFfrequency():
                standard=infinity [Hz]. Set the cut off frequency of the low-pass-filter for the derivative.

        setRefreshInterval():
                See explanation in Constructor. Possibility to alter the refresh rate during run-time.

        setControllerGains():
                See explanation in Constructor. Possibility to alter the controller gains during run-time.

        update():
                Routine, for updating the controller output with the input of the error and optionally, the derivative of the error.
                error: Input value for the proportional part and integral part. On faulty input, the input value is estimated with previous error-input values.
                derivative: Optional input value for the derivative part. If no value given, the derivative will be calculated with previous error-input values.

        updateSuccessRate():
                Gives out the percentage (0...1) value of valid error-input-values of the update()-member function. Helpful for analysing the quality of the input signal.
		

************************************************************/


class PIDcontroller {

public:
        // Constructor
        PIDcontroller (float Kp, float Ki, float Kd, const uint32_t dt,
                              const int8_t refreshInterval = 1);



        // Setting the Integral Limit
        //	Sets the absolute Value of the input value as the integral limit. Value should be the maximal output value.
        void setIntegralLimit (const float integralLimit);



        // Setting the Low-Pass-Filter cut-off-frequency
        void setLPFfrequency (const float freq);



        // Setting the update intervall
        void setRefreshInterval (const int8_t refreshInterval);



        // Adjusting the Controller Gains Proportional, Integral, Derivative
        void setControllerGains (const float Kp, const float Ki, const float Kd);



        // Update Routine.
        // 	If derivative not available, leave out this variable so not a number will be chosen and quasi-derivative is then calculated.
        float update (float error, const float derivative = NAN);



        // Reading the Update Success rate
        //	For analysis of the quality of the input signal
        float updateSuccessRate();



private:
        float m_fPrevOutput;		// Value of the last calculated Output Value
        float m_fPrevError[2];		// History of Error-Input-Values; [0]: oldest
        float m_fPrevUpdateError;	// Error-Value of the last update (for derivative)
        float m_fPrevDerivative;	// Value of the last derivative (for Low-Pass-Filter)
        float m_fAlpha;                 // Factor for LP-Filter: Indication of Filtering Treshold
        float m_fIntegral;      	// Actual Integral-Value
        float m_fIntegralLimit;		// Boundary of the Integral
        float m_fKp, m_fKi, m_fKd;	// Gain factors of porportional, integral, and derivative Part
        int8_t m_iRefreshInterval;	// Factor of the intervall time, the controller is updated
        int8_t m_iRefreshIntervalCounter;// Counter for the next execution of the controller
        uint32_t m_iDt;     		// Intervall time for update rate (for case refreshIntervall==1); time in microseconds
        uint32_t m_iUpdateCounter;	// Counter of sum of updated input error values
        uint32_t m_iBadInputCounter;	// Counting number of bad input error values


};


#include "PID.c"

#endif
