/**
 * Motors.h
 *
 * @author Øystein Schrøder Elvik
 * @version 1.0
 * @since 05.11.2014
 *
 * Inspiration:
 * http://www.instructables.com/id/RC-Quadrotor-Helicopter/step13/Arduino-Demo-PWM-Output/
 * arduino Servo lib
 * http://robots.dacloughb.com/project-3/quadcopter-software/motor-control/
 * https://github.com/2012SEng466/copter/blob/254ffeab74c9f0b5ef600415e1472f9756c2ecc5/Aero_Quad_X/Libraries/AQ_Motors/Motors_PWM_Timer.h
 * https://sites.google.com/site/qeewiki/books/avr-guide/pwm-on-the-atmega328
 * 
 */

#ifndef MOTORS_H_
#define MOTORS_H_

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#define RIGHT_MOTOR_PIN 9
#define LEFT_MOTOR_PIN 10
#define REAR_MOTOR_PIN 11
#define SERVO_PIN 3

#define PWM_FREQUENCY 300   // in Hz
#define PWM_PRESCALER 8
#define PWM_COUNTER_PERIOD (F_CPU/PWM_PRESCALER/PWM_FREQUENCY)

class Motors {
public:
  Motors();
  void init(int motors);
  void command(byte command[]);

private:
	int motors;
};


#endif