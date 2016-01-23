PIDcontroller::PIDcontroller (const float Kp, const float Ki, const float Kd, const uint32_t dt,
                      const int8_t updateIntervall)  :
                        m_fKp(Kp), m_fKi(Ki), m_fKd(Kd), m_iDt(dt) , m_iUpdateIntervall(updateIntervall) {

        m_fPrevOutput = 0;
        m_fPrevError[0] = 0;
        m_fPrevError[1] = 0;
        m_fIntegral = 0;
        m_iUpdateIntervallCounter = 0;
        m_iUpdateCounter = 0;
        m_iBadInputCounter = 0;
        m_fIntegralLimit = 100;
        m_fPrevUpdateError = 0;
        m_fPrevDerivative = 0;
        m_fAlpha = 0;

        if( m_iDt == 0) {
                //Fehlerroutine
        }
}



// Setting the Integral Limit
/*	Sets the absolute Value of the input value as the integral limit. Value should be the maximal output value.
*/
void PIDcontroller::setIntegralLimit (const float integralLimit) {
        m_fIntegralLimit = abs(integralLimit);
        return;
}



// Setting the Low-Pass-Filter cut-off-frequency
void PIDcontroller::setLPFfrequency (const float RC) {
        m_fAlpha = static_cast<float>(m_iDt) / (RC + static_cast<float>(m_iDt) );
        return;
}



// Setting the update intervall
void PIDcontroller::setUpdateIntervall (const int8_t updateIntervall) {
        m_iUpdateIntervall = updateIntervall;
        return;
}



// Adjusting the Controller Gains Proportional, Integral, Derivative
void PIDcontroller::setControllerGains (const float Kp, const float Ki, const float Kd) {

        m_fKp = Kp;
        m_fKi = Ki;
        m_fKd = Kd;
        return;

}



// Update Routine.
/* 	If derivative not available, leave out this variable so not a number will be chosen	and quasi-derivative is then calculated.
*/
float PIDcontroller::update (float error, const float derivative) {

        float t_fK,t_fLambda;
        float t_fOutput;
        float t_fDerivative;

        // Check for faulty input. If faulty, make an estimation of the error (exponentionally decreasing function)
        if (isnan(error)) {

                t_fK = m_fPrevError[1];
                t_fLambda = log(m_fPrevError[1]/m_fPrevError[0]);
                error = t_fK * exp(t_fLambda);		// Other approach: Set to zero <-------

                // increase counter for bad input
                m_iBadInputCounter++;

        }

        // Increase UpdateCounter and refresh the error history
        m_iUpdateCounter++;	// If longer operation than 2 years, check for overrun and also reset badInputCounter.
        m_fPrevError[0] = m_fPrevError[1];
        m_fPrevError[1] = error;

        // Check, if the Update should be executed or still waiting for the next intervall. If waiting, return the previous out-Value.
        if (m_iUpdateIntervallCounter > 0) {
                m_iUpdateIntervallCounter--;
                return(m_fPrevOutput);
        }

        // The case to recalculate the output has occurred
        // reset the IntervallCounter
        m_iUpdateIntervallCounter = m_iUpdateIntervall - 1;

        // Update Integral
        m_fIntegral = m_fIntegral + error * static_cast<float> (m_iDt)/1e6 * m_iUpdateIntervall;
        // Restrict Integral to boundaries
        if(m_fIntegral > m_fIntegralLimit){
                m_fIntegral = m_fIntegralLimit;
        }
        else if (m_fIntegral < -m_fIntegralLimit){
                m_fIntegral = -m_fIntegralLimit;
        }

        // Update derivative
        t_fDerivative = (error - m_fPrevUpdateError) / (static_cast<float> (m_iDt)/1e6 * m_iUpdateIntervall);
        m_fPrevUpdateError = error;

        // Low-Pass-Filtering of Derivative
        t_fDerivative = m_fAlpha * t_fDerivative + (1.0 - m_fAlpha) * m_fPrevDerivative;
        m_fPrevDerivative = t_fDerivative;

        // Calculate Output
        t_fOutput = m_fKp * error +	m_fKi * m_fIntegral + m_fKd * t_fDerivative;

        m_fPrevOutput = t_fOutput;

        return(t_fOutput);
}


// Reading the Update Success rate
/*	For analysis of the quality of the input signal
*/
float PIDcontroller::updateSuccessRate(){
        if(m_iUpdateCounter==0){return(0);}
        else{return(static_cast<float>(m_iBadInputCounter)/static_cast<float>(m_iUpdateCounter));}
}
