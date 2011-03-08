/**
 * Transfere attitude data compatible with Happy Killmore ground station
 * 
 * Only if HK_ENABLED = 1
 */
void HappyKillmoreSendAttitude(){
  if(bitRead(config[CV_TRICOPTER_ENABLE_BYTE], CV_HK_ENABLE_BIT) == 1){
    Serial.print("+++"); //Prefix
    Serial.print("ASP:"); //Airspeed
    Serial.print(20, DEC);
    Serial.print(",THH:"); //Throttle
    Serial.print(map(receiver.getThro(),153,862,0,100), DEC);
    Serial.print (",RLL:"); //Roll
    Serial.print(imu.getRollDegree());//(analogRead(ACC_ROLL_PIN) - init_acc_roll) * acc_scale, DEC);
    Serial.print (",PCH:"); //Pitch
    Serial.print(imu.getNickDegree());
    Serial.println(",***"); //Suffix
  }
}

/**
 * Transfere location data compatible with Happy Killmore ground station
 * 
 * Only if HK_ENABLED = 1
 */
void HappyKillmoreSendLocation(){
  if(bitRead(config[CV_TRICOPTER_ENABLE_BYTE], CV_HK_ENABLE_BIT) == 1){
    Serial.print("!!!"); //Prefix
    Serial.print("LAT:"); //Latitude
    Serial.print("67967300");//"33952600");
    Serial.print(",LON:"); //Longitude
    Serial.print("14994700");//"-117409072");
    Serial.print(",SPD:"); //Speed over ground from GPS
    Serial.print(30);
    Serial.print(",CRT:"); //Climb rate m/s
    Serial.print("100");
    Serial.print(",ALT:"); //Altitude in meters
    Serial.print(1000);
    Serial.print(",ALH:"); //Target Altitude
    Serial.print("450");
    Serial.print(",CRS:"); //Course over ground in degrees
    Serial.print(imu.getYawDegree()); 
    Serial.print(",BER:"); //Bearing (target heading)
    Serial.print("94");
    Serial.print(",WPN:"); //Waypoint number
    Serial.print("0");
    Serial.print(",DST:"); //Distance from Waypoint
    Serial.print("25853");
    Serial.print(",BTV:"); //Battery voltage
    Serial.print("11840");
    Serial.println(",***"); //suffix
  }
}
