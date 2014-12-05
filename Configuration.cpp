/*
 * Configuration.cpp
 *
 * author: Øystein Schrøder Elvik
 */

#include "Configuration.h"

#include <EEPROM.h>

Configuration::Configuration(int eepromStart){
	this->eepromStart = eepromStart;
	dirty = true;

	readEEPROM();
}

byte Configuration::get(byte address){
	return config[address];
}

void Configuration::set(byte conf[]){
	for(int i=0; i < CV_END_BYTE + 1; i++) config[i]=conf[i];

	writeEEPROM();
	
	dirty = true;
}

void Configuration::reset(){
	byte config[CV_END_BYTE + 1];
	config[CV_TRICOPTER_ENABLE_BYTE] = 255; //Enable all

	config[CV_LEFT_MOTOR_PIN_BYTE] = 11;
	config[CV_RIGHT_MOTOR_PIN_BYTE] = 10;
	config[CV_REAR_MOTOR_PIN_BYTE] = 9;
	config[CV_YAW_SERVO_PIN_BYTE] = 3;
	config[CV_MIN_THRO_BYTE] = 50;
	config[CV_ESC_ARM_BYTE] = 22;
	config[CV_MOTOR_SPIN_UP_BYTE] = 50;
	config[CV_HOVER_PID_KP_BYTE] = 24; // Kp = var / 25 (0-10.2)  //17
	config[CV_HOVER_PID_KI_BYTE] = 18; //Ki = var / 255 (0-1)      //7
	config[CV_HOVER_PID_KD_BYTE] = 60; //Kp = var / 25 (0-10.2)  //165
	config[CV_ACRO_PID_KP_BYTE] = 20; // Kp = var / 25 (0-10.2)   //20
	config[CV_ACRO_PID_KI_BYTE] = 4; //Ki = var / 255 (0-1)       //1
	config[CV_ACRO_PID_KD_BYTE] = 45; //Kp = var / 25 (0-10.2)     //0
	config[CV_YAW_PID_KP_BYTE] = 20; // Kp = var / 25 (0-10.2)    //22
	config[CV_YAW_PID_KI_BYTE] = 4; //Ki = var / 255 (0-1)        //112
	config[CV_YAW_PID_KD_BYTE] = 45; //Kp = var / 25 (0-10.2)      //0

	//------ Receiver ---------
	config[CV_RX_REVERSING_BYTE] = 12;

	//------ IMU --------
	config[CV_IMU_GYRO_ROLL_PIN_BYTE] = 4;
	config[CV_IMU_GYRO_NICK_PIN_BYTE] = 3;
	config[CV_IMU_GYRO_YAW_PIN_BYTE] = 5;
	config[CV_IMU_ACC_ROLL_PIN_BYTE] = 1;
	config[CV_IMU_ACC_NICK_PIN_BYTE] = 2;
	config[CV_IMU_ACC_VERT_PIN_BYTE] = 0;
	//Reversing
	config[CV_IMU_REVERSING_BYTE] = 40; //00101000
	//Accelerometer trim
	config[CV_IMU_ACC_ROLL_TRIM_BYTE] = 127; //trim = (val * 4) - 511
	config[CV_IMU_ACC_NICK_TRIM_BYTE] = 127;
	config[CV_IMU_ACC_VERT_TRIM_BYTE] = 127;
	//Accelerometer gain
	config[CV_IMU_ACC_WEIGHT_BYTE] = 40;

	set(config);
}

void Configuration::readEEPROM(){
	for(int i = 0; i < CV_END_BYTE + 1; i++){
		config[i] = EEPROM.read(i + eepromStart);
	}

	dirty = true;
}
void Configuration::writeEEPROM(){
	for(int i = 0; i < CV_END_BYTE + 1; i++){
		EEPROM.write(i + eepromStart, config[i]);
	}
}
