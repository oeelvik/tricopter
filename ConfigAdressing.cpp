//############ Configuration Var Adressing #################

//------------ Tricopter --------------
#define CV_TRICOPTER_ENABLE_BYTE 0 //Address to byte containing boolean bits
  #define CV_MOTORS_ENABLE_BIT 0
  #define CV_YAW_SERVO_REV_BIT 1
  #define CV_PID_ENABLE_BIT 3
  #define CV_HK_ENABLE_BIT 6
  #define CV_TRIGUI_ENABLE_BIT 7
#define CV_LEFT_MOTOR_PIN_BYTE 1
#define CV_RIGHT_MOTOR_PIN_BYTE 2
#define CV_REAR_MOTOR_PIN_BYTE 3
#define CV_YAW_SERVO_PIN_BYTE 4
#define CV_MIN_THRO_BYTE 5 //Minimum trottle to activate motors and pid integration
#define CV_MIN_ESC_BYTE 6 //Lowest value written to esc
#define CV_PID_SAMPLE_TIME_BYTE 7 
#define CV_PID_KP_BYTE 8
#define CV_PID_KI_BYTE 9
#define CV_PID_KD_BYTE 10

//------ Receiver ---------
#define CV_RX_REVERSING_BYTE 11
  #define CV_RX_THRO_REV_BIT 0
  #define CV_RX_AILE_REV_BIT 1
  #define CV_RX_ELEV_REV_BIT 2
  #define CV_RX_RUDD_REV_BIT 3
  #define CV_RX_GEAR_REV_BIT 4
  #define CV_RX_FLAP_REV_BIT 5
  
//------ IMU --------
//Pins
#define CV_IMU_GYRO_ROLL_PIN_BYTE 12
#define CV_IMU_GYRO_NICK_PIN_BYTE 13
#define CV_IMU_GYRO_YAW_PIN_BYTE 14
#define CV_IMU_ACC_ROLL_PIN_BYTE 15
#define CV_IMU_ACC_NICK_PIN_BYTE 16
#define CV_IMU_ACC_VERT_PIN_BYTE 17
//Reversing
#define CV_IMU_REVERSING_BYTE 18
  #define CV_IMU_GYRO_ROLL_REV_BIT 0
  #define CV_IMU_GYRO_NICK_REV_BIT 1
  #define CV_IMU_GYRO_YAW_REV_BIT 2
  #define CV_IMU_ACC_ROLL_REV_BIT 3
  #define CV_IMU_ACC_NICK_REV_BIT 4
  #define CV_IMU_ACC_VERT_REV_BIT 5
//Accelerometer trim
#define CV_IMU_ACC_ROLL_TRIM_BYTE 19
#define CV_IMU_ACC_NICK_TRIM_BYTE 20
#define CV_IMU_ACC_VERT_TRIM_BYTE 21
//Accelerometer gain
#define CV_IMU_ACC_GAIN_BYTE 22

#define CV_END_BYTE 22



