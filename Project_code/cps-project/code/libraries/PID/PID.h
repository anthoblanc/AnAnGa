#ifndef PID_H
#define PID_H

float rollPIDOut = 0;
float pitchPIDOut = 0;
float speedPIDOut = 0;

// to be added:
//	Integral maximum -> bounded value of integral,
//	low pass filter for derivative? -> limited value?
//	skipping not defined input values. 			check!

class PIDcontroller {

	public:
		PIDcontroller (const float Kp, const float Ki, const float Kd, const uint32_t dt, 
				      const uint8_t updateIntervall)  : 
					m_fKp(Kp), m_fKi(Ki), m_fKd(Kd), m_iDt(dt) , m_iUpdateIntervall(updateIntervall){
	
			m_fPrevOutput = 0;			
			m_fPrevError = 0;
			m_fIntegral = 0;
			m_iUpdateIntervallCounter = 0;
			m_iUpdateCounter = 0;
			m_iUpdateSucceeded = 0;
	
			if( m_iDt == 0) {
				//Fehlerroutine	
			}
		}

		float update (const float error, const float derivative) {
			
			if (m_iUpdateIntervallCounter > 0){
				m_iUpdateIntervallCounter--;
				return(m_fPrevOutput);
			}else{
			
				m_iUpdateIntervallCounter = m_iUpdateIntervall - 1;
				m_iUpdateCounter++;
				if (isnan(error) || isnan(derivative)){
					return(0);
				}
				m_iUpdateSucceeded++;

				float t_fOutput;
		
				m_fIntegral = m_fIntegral + error * static_cast<float> (m_iDt);
				if(m_fIntegral > 100){
					m_fIntegral = 100;
				}
				else if (m_fIntegral < -100){
					m_fIntegral = -100;
				}
		
				t_fOutput = m_fKp * error +
							m_fKi * m_fIntegral +
							m_fKd * derivative;
		
				m_fPrevOutput = t_fOutput;

				return(t_fOutput);
				
			}
		}
		
		float updateWODer (const float error) {
			m_iUpdateCounter++;
			if (isnan(error)){
				return(0);
			}
			m_iUpdateSucceeded++;
	
			float t_fDerivative;
			float t_fOutput;
	
			m_fIntegral = m_fIntegral + error * static_cast<float> (m_iDt);
			if(m_fIntegral > 100){
				m_fIntegral = 100;
			}
			else if (m_fIntegral < -100){
				m_fIntegral = -100;
			}
			
			t_fDerivative = (error - m_fPrevError) / static_cast<float> (m_iDt);
			// Begrenzen?
	
			t_fOutput = m_fKp * error +
						m_fKi * m_fIntegral +
						m_fKd * t_fDerivative;
	
			m_fPrevError = error;
			m_fPrevOutput = t_fOutput;

			return(t_fOutput);
		}
		
		float updateSuccessRate(){
			if(m_iUpdateCounter==0){return(0);}
			else{return(static_cast<float>(m_iUpdateSucceeded)/static_cast<float>(m_iUpdateCounter));}
		}
	
	private:
		float m_fPrevOutput;
		float m_fPrevError;
		float m_fIntegral;
		float m_fKp, m_fKi, m_fKd;
		uint8_t m_iUpdateIntervall;
		uint8_t m_iUpdateIntervallCounter;
		uint32_t m_iDt;	
		uint32_t m_iUpdateCounter;
		uint32_t m_iUpdateSucceeded;

};



#endif
