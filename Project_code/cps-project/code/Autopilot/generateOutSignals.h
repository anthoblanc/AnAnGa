#ifndef GENERATEOUTSIGNALS_H
#define GENERATEOUTSIGNALS_H

/***********************************************************

generateOutSignals.h
	Final processing of the steering signals. Conversion from float to a signal, suitable for the actuators of the airplane.
	(Initially given).

************************************************************/

void generateOutSignals(const struct SteeringSignals stSig, int16_t &aileronLOut, int16_t &aileronROut, int16_t &elevatorLOut, int16_t &elevatorROut, int16_t &throttleOut, int16_t &rudderOut){

    // Constrain all control surface outputs to the range -1 to 1
    float aileronL = -1 * constrain(stSig.aileron, -1, 1);
    float aileronR = constrain(stSig.aileron, -1, 1);
    float elevatorL = constrain(stSig.elevator, -1, 1);
    float elevatorR = constrain(stSig.elevator, -1, 1);
    float throttle = constrain(stSig.throttle, 0, 1);
    float rudder = constrain(stSig.rudder, -1, 1);

    #define SERVO_MIN 1000 // Minimum duty cycle
    #define SERVO_MID 1500 // Mid duty cycle
    #define SERVO_MAX 2000 // Maximum duty cycle
    // Compute duty cycle for PWM output from generic control
    aileronLOut = ((aileronL+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
    aileronROut = ((aileronR+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
    elevatorLOut = ((elevatorL+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
    elevatorROut = ((elevatorR+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;
    throttleOut = ((throttle)*(SERVO_MAX-SERVO_MIN)) + SERVO_MIN;
    rudderOut = ((rudder+1.0)*(SERVO_MAX-SERVO_MIN)/2.0) + SERVO_MIN;

}

#endif
