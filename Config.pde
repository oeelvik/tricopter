void setConfig(int conf[]){
  for(int i=0; i < CV_END_BYTE + 1; i++) config[i]=conf[i];
  writeEEPROMConfig();
  reloade();
}

void readEEPROMConfig(){
  for(int i = 0; i < CV_END_BYTE + 1; i++){
    config[i] = EEPROM.read(i);
  }
}
void writeEEPROMConfig(){
  for(int i = 0; i < CV_END_BYTE + 1; i++){
    EEPROM.write(i, config[i]);
  }
}

void resetConfig(){
  int config[CV_END_BYTE + 1];
  config[CV_TRICOPTER_ENABLE_BYTE] = 255; //Enable all
  
  config[CV_LEFT_MOTOR_PIN_BYTE] = 11;
  config[CV_RIGHT_MOTOR_PIN_BYTE] = 10;
  config[CV_REAR_MOTOR_PIN_BYTE] = 9;
  config[CV_YAW_SERVO_PIN_BYTE] = 3;
  config[CV_MIN_THRO_BYTE] = 50; // minThro = val * 4
  config[CV_MIN_ESC_BYTE] = 7; // minESC = val * 1.416667 eller map(val, 0, 255, 0, 179)
  config[CV_PID_SAMPLE_TIME_BYTE] = 5; //Sampletime = val * 4 (0-1024 millis)
  config[CV_PID_KP_BYTE] = 25; // Kp = var / 25 (0-10.2) 
  config[CV_PID_KI_BYTE] = 12; //Ki = var / 255 (0-1)
  config[CV_PID_KD_BYTE] = 40; //Kp = var / 50 (0-5.1)
  
  //------ Receiver ---------
  config[CV_RX_REVERSING_BYTE] = 0;
    
  //------ IMU --------
  config[CV_IMU_GYRO_ROLL_PIN_BYTE] = 4;
  config[CV_IMU_GYRO_NICK_PIN_BYTE] = 3;
  config[CV_IMU_GYRO_YAW_PIN_BYTE] = 5;
  config[CV_IMU_ACC_ROLL_PIN_BYTE] = 1;
  config[CV_IMU_ACC_NICK_PIN_BYTE] = 2;
  config[CV_IMU_ACC_VERT_PIN_BYTE] = 0;
  //Reversing
  config[CV_IMU_REVERSING_BYTE] = 7; //00000111
  //Accelerometer trim
  config[CV_IMU_ACC_ROLL_TRIM_BYTE] = 127-3; //trim = (val * 4) - 511
  config[CV_IMU_ACC_NICK_TRIM_BYTE] = 127-3;
  config[CV_IMU_ACC_VERT_TRIM_BYTE] = 127+10;
  //Accelerometer gain
  config[CV_IMU_ACC_GAIN_BYTE] = 4; //gain = val / 100
  
  setConfig(config);
}