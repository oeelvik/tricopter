class Tricopter implements SerialSerializable {
  
  public RCReceiver receiver = new RCReceiver();
  public IMU imu = new IMU();
  public Config config = new Config();
  
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

  int[] toSerialStreem(int CMD) {
    //-------- Data ------------
    data[0] = time;
    //fastLoopCounter;
    data[1] = state;
    data[2] = mode;
    
    //Motors and servo:
    data[3] = leftMotor;
    data[4] = rightMotor;
    data[5] = rearMotor;
    data[6] = yawServo;
    
    //PID
    data[7] = PIDRoll;
    data[8] = PIDNick;
    data[9] = PIDYaw;
    return data;
  }

  void parseSerialStreem(int CMD, int[] data) {
    //-------- Data ------------
    time = data[0];
    //fastLoopCounter;
    state = data[1];
    mode = data[2];
    
    //Motors and servo:
    leftMotor = data[3];
    rightMotor = data[4];
    rearMotor = data[5];
    yawServo = data[6];
    
    //PID
    PIDRoll = data[7];
    PIDNick = data[8];
    PIDYaw = data[9];
    this.isUpdated = true;
  }

  class RCReceiver implements SerialSerializable {
    public int thro = 0;
    public int aile = 0;
    public int elev = 0;
    public int rudd = 0;
    public int gear = 0;
    public int flap = 0;
    
    int[] data = new int[12];


    int[] toSerialStreem(int CMD) {
      data[0] = thro;
      data[1] = aile;
      data[2] = elev;
      data[3] = rudd;
      data[4] = gear;
      data[5] = flap;
      return data;
    }

    void parseSerialStreem(int CMD, int[] data) {
      thro = data[0];
      aile = data[1];
      elev = data[2];
      rudd = data[3];
      gear = data[4];
      flap = data[5];
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
    
    int[] data = new int[19];
    
    int[] toSerialStreem(int CMD) {
      data[0] = roll;
      data[1] = nick;
      data[2] = yaw;
      
      data[3] = gRoll;
      data[4] = gNick;
      data[5] = gYaw;
      
      data[6] = aRoll;
      data[7] = aNick;
      data[8] = aVert;
      return data;
    }

    void parseSerialStreem(int CMD, int[] data) {
      roll = data[0];
      nick = data[1];
      yaw = data[2];
      
      gRoll = data[3];
      gNick = data[4];
      gYaw = data[5];
      
      aRoll = data[6];
      aNick = data[7];
      aVert = data[8];
    }
  }
  
  class Config implements SerialSerializable {
    
    public boolean currentSetupReceived = false;
    public boolean loadedToGUI = false;
    
    public int[] data = new int[CV_END_BYTE + 1];
    
    int[] toSerialStreem(int CMD) {
      if(CMD == 5){
        int [] var = new int[0];
        return var;
      } else return data;
    }

    void parseSerialStreem(int CMD, int[] data) {
      this.currentSetupReceived = true;
      this.loadedToGUI = false;
      
      for(int i = 0; i < CV_END_BYTE + 1; i++){
        this.data[i] = data[i];
      }
    }
  }
}
