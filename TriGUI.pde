void TriGUIsendMessage(int type, String message){
    Serial.print(">>>"); //Prefix
    Serial.write(1); //CMD message
    Serial.write(type); //type
    Serial.print(message);
    Serial.println("***");
}

void TriGUIsendConfig(){
  Serial.print(">>>"); //Prefix
  Serial.write(5); //CMD Tricopter
  for(int i=0; i < sizeof(config); i++) Serial.write(config[i]);  
  Serial.println("***"); //Suffix
  TriGUIsendMessage(0, "Configuration sendt form tricopter");
}

void TriGUIsendCopter(){
  if(bitRead(config[CV_TRICOPTER_ENABLE_BYTE], CV_TRIGUI_ENABLE_BIT) == 1){
    //Tricopter
    Serial.print(">>>"); //Prefix
    Serial.write(2); //CMD Tricopter
    
    //--------- Data --------------
    Serial.write(millis() / 10000); //time 0 - ca 45 min
    Serial.write(((receiver.getThro() > config[CV_MIN_THRO_BYTE] * 4)? 0x30 : 0x20));
    Serial.write(mode);
   
    //Motors and servo
    Serial.write(map(mix.getLeftThrust(), 0, 179, 0, 255));
    Serial.write(map(mix.getRightThrust(), 0, 179, 0, 255));
    Serial.write(map(mix.getRearThrust(), 0, 179, 0, 255));
    Serial.write(map(mix.getYawPos(), 0, 179, 0, 255));
    
    //PID
    Serial.write(((int)rollOutput + 1023) / 8);
    Serial.write(((int)nickOutput + 1023) / 8);
    Serial.write(((int)yawOutput + 1023) / 8);
    
    Serial.println("***"); //Suffix
  }
}

void TriGUIsendReceiver(){
  if(bitRead(config[CV_TRICOPTER_ENABLE_BYTE], CV_TRIGUI_ENABLE_BIT) == 1){
    Serial.print(">>>"); //Prefix
    Serial.write(3); //CMD receiver
    
    //Signal
    Serial.write(receiver.getThro() / 4);
    Serial.write(receiver.getAile() / 4);
    Serial.write(receiver.getElev() / 4);
    Serial.write(receiver.getRudd() / 4);
    Serial.write(receiver.getGear() / 4);
    Serial.write(receiver.getFlap() / 4);
    
    Serial.println("***"); //Suffix
  }
}

void TriGUIsendIMU(){
  if(bitRead(config[CV_TRICOPTER_ENABLE_BYTE], CV_TRIGUI_ENABLE_BIT) == 1){
    Serial.print(">>>"); //Prefix
    Serial.write(4); //CMD IMU
    
    //Signal
    Serial.write(map(imu.getRollDegree(), -180, 180, 0, 255));
    Serial.write(map(imu.getNickDegree(), -180, 180, 0, 255));
    Serial.write(map(imu.getYawDegree(), -180, 180, 0, 255));
    
    Serial.write(0);//imu.getGyroRoll() / 8 + 127);
    Serial.write(0);//imu.getGyroNick() / 8 + 127);
    Serial.write(0);//imu.getGyroYaw() / 8 + 127);
    
    Serial.write(0);//imu.getAccRoll() / 4);
    Serial.write(0);//imu.getAccNick() / 4);
    Serial.write(0); // AccVert //TODO: not implemented in IMU class
    
    Serial.println("***"); //Suffix
  }
}
