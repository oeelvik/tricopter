void TriGUIsendMessage(int type, String message){
    Serial.print(">>>"); //Prefix
    Serial.print(1,BYTE); //CMD message
    Serial.print(type,BYTE); //type
    Serial.print(message);
    Serial.println("***");
}

void TriGUIsendConfig(){
  Serial.print(">>>"); //Prefix
  Serial.print(5, BYTE); //CMD Tricopter
  for(int i=0; i < sizeof(config); i++) Serial.print(config[i], BYTE);  
  Serial.println("***"); //Suffix
  TriGUIsendMessage(0, "Configuration sendt form tricopter");
}

void TriGUIsendCopter(){
  //Tricopter
    Serial.print(">>>"); //Prefix
    Serial.print(2, BYTE); //CMD Tricopter
    
    //--------- Data --------------
    Serial.print(millis() / 10000,BYTE); //time 0 - ca 45 min
    Serial.print(((receiver.getThro() > config[CV_MIN_THRO_BYTE] * 4)? 0x30 : 0x20), BYTE);
    Serial.print(((receiver.getFlap() < RXCENTER) ? 0x00: 0x10), BYTE);
   
    //Motors and servo
    Serial.print(map(mix.getLeftThrust(), 0, 179, 0, 255),BYTE);
    Serial.print(map(mix.getRightThrust(), 0, 179, 0, 255),BYTE);
    Serial.print(map(mix.getRearThrust(), 0, 179, 0, 255),BYTE);
    Serial.print(map(mix.getYawPos(), 0, 179, 0, 255),BYTE);
    
    //PID
    Serial.print((rollForce + 1023) / 8,BYTE);
    Serial.print((nickForce + 1023) / 8,BYTE);
    Serial.print((yawForce + 1023) / 8,BYTE);
    
    Serial.println("***"); //Suffix
    
}

void TriGUIsendReceiver(){
  Serial.print(">>>"); //Prefix
  Serial.print(3,BYTE); //CMD receiver
  
  //Signal
  Serial.print(receiver.getThro() / 4,BYTE);
  Serial.print(receiver.getAile() / 4,BYTE);
  Serial.print(receiver.getElev() / 4,BYTE);
  Serial.print(receiver.getRudd() / 4,BYTE);
  Serial.print(receiver.getGear() / 4,BYTE);
  Serial.print(receiver.getFlap() / 4,BYTE);
  
  Serial.println("***"); //Suffix
    
}

void TriGUIsendIMU(){
  Serial.print(">>>"); //Prefix
  Serial.print(4,BYTE); //CMD IMU
  
  //Signal
  Serial.print(imu.getRoll() / 4,BYTE);
  Serial.print(imu.getNick() / 4,BYTE);
  Serial.print(imu.getYaw() / 4,BYTE);
  
  Serial.print(imu.getGyroRoll() / 8 + 127,BYTE);
  Serial.print(imu.getGyroNick() / 8 + 127,BYTE);
  Serial.print(imu.getGyroYaw() / 8 + 127,BYTE);
  
  Serial.print(imu.getAccRoll() / 4,BYTE);
  Serial.print(imu.getAccNick() / 4,BYTE);
  Serial.print(0,BYTE); // AccVert //TODO: not implemented in IMU class
  
  Serial.println("***"); //Suffix
}
