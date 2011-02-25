class Tricopter implements SerialSerializable {
  
  public RCReceiver receiver = new RCReceiver();
  public IMU imu = new IMU();
  
  //-------- Setup -----------
  public boolean enableMotors = false;
  public int minThrottle = 200; //Minimum value from receiver before motors starts
  public int minESC = 10; //The required minimum to arm esc
  
  //Serial communication
  public boolean HKEnable = false; //Turn on serial com for Happy Killmore Ground Station
  public boolean TriGUITransmitt = false; //Turn on serial transmitting af data from tricopter to TriGUI Config values is always transmitted
  
  //PID
  public boolean PIDEnable = true;
  public int PIDSampleTime = 20;
  public float PIDKp = 100;//1;
  public float PIDKi = 100;//0.05;
  public float PIDKd = 100;//0.8;
  
  
  //-------- Data ------------
  public int time = 0; //Millis on tricopter
  //public long fastLoopCounter = 0;
  public int state = 0x00; //0x00 = off, 0x10 = configuring, 0x20 = armed, 0x30 = airbourne, 0xA0 = Error
  public int mode = 0x00; //0x00 = hover, 0x10 = acro
  
  //Motors and servo:
  public int leftMotor = 0;
  public int rightMotor = 0;
  public int rearMotor = 0;
  public int yawServo = 0;
  
  //PID
  public int PIDRoll = 0;
  public int PIDNick = 0;
  public int PIDYaw = 0;
  
  
  int[] data = new int[20];
  public boolean isUpdated;
  
  Tricopter() {
  }

  int[] toSerialStreem() {
    //----------- Setup -----------
    data[0] = (enableMotors) ? 1:0;
    data[1] = minThrottle / 4;
    data[2] = minESC / 4;
    
    //Serial communication
    data[3] = (HKEnable) ? 1:0;
    data[4] = (TriGUITransmitt) ? 1:0;
    
    //PID
    data[5] = (PIDEnable) ? 1:0;
    data[6] = PIDSampleTime;
    data[7] = (int)(PIDKp);// * 25); //TODO: set faktor to represent the best range of values
    data[8] = (int)(PIDKi);// * 1000);
    data[9] = (int)(PIDKd);// * 100);
    
    //-------- Data ------------
    data[10] = time;
    //fastLoopCounter;
    data[11] = state;
    data[12] = mode;
    
    //Motors and servo:
    data[13] = leftMotor;
    data[14] = rightMotor;
    data[15] = rearMotor;
    data[16] = yawServo;
    
    //PID
    data[17] = PIDRoll;
    data[18] = PIDNick;
    data[19] = PIDYaw;
    return data;
  }

  void parseSerialStreem(int[] data) {
    //----------- Setup -----------
    enableMotors = (data[0] == 1);
    minThrottle = data[1] * 4;
    minESC = data[2] * 4;
    
    //Serial communication
    HKEnable = (data[3] == 1);
    TriGUITransmitt = (data[4] == 1);
    
    //PID
    PIDEnable = (data[5] == 1);
    PIDSampleTime = data[6];
    PIDKp = (float)data[7];// / 25; //TODO: set faktor to represent the best range of values
    PIDKi = (float)data[8];// / 1000;
    PIDKd = (float)data[9];// / 100;
    
    //-------- Data ------------
    time = data[10];
    //fastLoopCounter;
    state = data[11];
    mode = data[12];
    
    //Motors and servo:
    leftMotor = data[13];
    rightMotor = data[14];
    rearMotor = data[15];
    yawServo = data[16];
    
    //PID
    PIDRoll = data[17];
    PIDNick = data[18];
    PIDYaw = data[19];
    
    isUpdated = true;
  }

  class RCReceiver implements SerialSerializable {
    public int thro = 0;
    public int aile = 100;
    public int elev = 0;
    public int rudd = 0;
    public int gear = 0;
    public int flap = 0;

    public int thro_rev = 0;
    public int aile_rev = 0;
    public int elev_rev = 0;
    public int rudd_rev = 0;
    public int gear_rev = 0;
    public int flap_rev = 0;

    int[] data = new int[12];


    int[] toSerialStreem() {
      data[0] = thro;
      data[1] = aile;
      data[2] = elev;
      data[3] = rudd;
      data[4] = gear;
      data[5] = flap;

      data[6] = thro_rev;
      data[7] = aile_rev;
      data[8] = elev_rev;
      data[9] = rudd_rev;
      data[10] = gear_rev;
      data[11] = flap_rev;
      return data;
    }

    void parseSerialStreem(int[] data) {
      thro = data[0];
      aile = data[1];
      elev = data[2];
      rudd = data[3];
      gear = data[4];
      flap = data[5];

      thro_rev = data[6];
      aile_rev = data[7];
      elev_rev = data[8];
      rudd_rev = data[9];
      gear_rev = data[10];
      flap_rev = data[11];
    }
  }

  class IMU implements SerialSerializable {
    public int roll = 0;
    public int nick = 0;
    public int yaw = 0;
    
    public int gRoll = 0;
    public int gNick = 0;
    public int gYaw = 0;
    
    public int aRoll = 0;
    public int aNick = 0;
    public int aVert = 0;
    
    public boolean gRollRev = false;
    public boolean gNickRev = false;
    public boolean gYawRev = false;
    
    public boolean aRollRev = false;
    public boolean aNickRev = false;
    public boolean aVertRev = false;
    
    public int aRollTrim = 0;
    public int aNickTrim = 0;
    public int aVertTrim = 0;
    
    public float aGain = 0;
    
    int[] data = new int[19];
    
    int[] toSerialStreem() {
      data[0] = roll;
      data[1] = nick;
      data[2] = yaw;
      
      data[3] = gRoll;
      data[4] = gNick;
      data[5] = gYaw;
      
      data[6] = aRoll;
      data[7] = aNick;
      data[8] = aVert;
      
      data[9] = (gRollRev) ? 1:0;
      data[10] = (gNickRev) ? 1:0;
      data[11] = (gYawRev) ? 1:0;
      
      data[12] = (aRollRev) ? 1:0;
      data[13] = (aNickRev) ? 1:0;
      data[14] = (aVertRev) ? 1:0;
      
      data[15] = aRollTrim;
      data[16] = aNickTrim;
      data[17] = aVertTrim;
      
      data[18] = (int)(aGain * 100);
      return data;
    }

    void parseSerialStreem(int[] data) {
      roll = data[0];
      nick = data[1];
      yaw = data[2];
      
      gRoll = data[3];
      gNick = data[4];
      gYaw = data[5];
      
      aRoll = data[6];
      aNick = data[7];
      aVert = data[8];
      
      gRollRev = (data[9] == 1);
      gNickRev = (data[10] == 1);
      gYawRev = (data[11] == 1);
      
      aRollRev = (data[12] == 1);
      aNickRev = (data[13] == 1);
      aVertRev = (data[14] == 1);
      
      aRollTrim = data[15];
      aNickTrim = data[16];
      aVertTrim = data[17];
      
      aGain = (float) data[18] / 100;
    }
  }
}
