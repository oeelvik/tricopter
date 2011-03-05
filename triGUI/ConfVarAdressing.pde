
//------------ Tricopter --------------
final int CV_TRICOPTER_ENABLE_BYTE = 0; //Address to byte containing boolean bits
  final int CV_MOTORS_ENABLE_BIT = 0;
  final int CV_YAW_SERVO_REV_BIT = 1;
  final int CV_PID_ENABLE_BIT = 3;
  final int CV_HK_ENABLE_BIT = 6;
  final int CV_TRIGUI_ENABLE_BIT = 7;
final int CV_LEFT_MOTOR_PIN_BYTE = 1;
final int CV_RIGHT_MOTOR_PIN_BYTE = 2;
final int CV_REAR_MOTOR_PIN_BYTE = 3;
final int CV_YAW_SERVO_PIN_BYTE = 4;
final int CV_MIN_THRO_BYTE = 5; //Minimum trottle to activate motors and pid integration
final int CV_MIN_ESC_BYTE = 6; //Lowest value written to esc
final int CV_PID_SAMPLE_TIME_BYTE = 7; 
final int CV_PID_KP_BYTE = 8;
final int CV_PID_KI_BYTE = 9;
final int CV_PID_KD_BYTE = 10;

//------ Receiver ---------
final int CV_RX_REVERSING_BYTE = 11;
  final int CV_RX_THRO_REV_BIT = 0;
  final int CV_RX_AILE_REV_BIT = 1;
  final int CV_RX_ELEV_REV_BIT = 2;
  final int CV_RX_RUDD_REV_BIT = 3;
  final int CV_RX_GEAR_REV_BIT = 4;
  final int CV_RX_FLAP_REV_BIT = 5;
  
//------ IMU --------
//Pins
final int CV_IMU_GYRO_ROLL_PIN_BYTE = 12;
final int CV_IMU_GYRO_NICK_PIN_BYTE = 13;
final int CV_IMU_GYRO_YAW_PIN_BYTE = 14;
final int CV_IMU_ACC_ROLL_PIN_BYTE = 15;
final int CV_IMU_ACC_NICK_PIN_BYTE = 16;
final int CV_IMU_ACC_VERT_PIN_BYTE = 17;
//Reversing
final int CV_IMU_REVERSING_BYTE = 18;
  final int CV_IMU_GYRO_ROLL_REV_BIT = 0;
  final int CV_IMU_GYRO_NICK_REV_BIT = 1;
  final int CV_IMU_GYRO_YAW_REV_BIT = 2;
  final int CV_IMU_ACC_ROLL_REV_BIT = 3;
  final int CV_IMU_ACC_NICK_REV_BIT = 4;
  final int CV_IMU_ACC_VERT_REV_BIT = 5;
//Accelerometer trim
final int CV_IMU_ACC_ROLL_TRIM_BYTE = 19;
final int CV_IMU_ACC_NICK_TRIM_BYTE = 20;
final int CV_IMU_ACC_VERT_TRIM_BYTE = 21;
//Accelerometer gain
final int CV_IMU_ACC_GAIN_BYTE = 22;

final int CV_END_BYTE = 22;


boolean bitRead(int b, int index){
  b >>= index;
  return (( b & 0x01) > 0);
}

int bitWrite(int b, int index, boolean val){
  if(val){
    return (byte)b | (byte)pow(2, index);
  } else {
    return (byte)b & (byte)(255 - pow(2, index));
  }
}
