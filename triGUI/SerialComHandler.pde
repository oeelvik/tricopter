interface SerialSerializable {
  int[] toSerialStreem(int CMD);
  void parseSerialStreem(int CMD, int[] data);
}

class SerialComHandler {
  final int BYTE_LIMIT = 50;

  HashMap serializableObjects = new HashMap();

  int CMD;
  int[] data = new int[BYTE_LIMIT];

  boolean inMessage = false;
  int messageByteCount = 0;
  int postFixCount = 0;
  Serial serial;
  
  void setSerial(Serial serial){
    this.serial = serial;
  }

  void addObject(int CMD, SerialSerializable object) {
    serializableObjects.put(CMD, object);
  }

  void receive(int inByte) {
    messageByteCount++;

    if(!inMessage  && inByte != '>') { //Not this protocol
      messageByteCount = 0;
    } 
    else if(inByte == '>' && messageByteCount == 3) { //prefix received
      inMessage = true;
      postFixCount = 0;
      this.CMD = 0;
    } 
    else if (inMessage) {

      if(inByte == '*') postFixCount++;
      else postFixCount = 0;
      if(postFixCount == 3 || messageByteCount - 5 >= BYTE_LIMIT) { //All bytes in message reseived
        inMessage = false;
        callReceiver(this.CMD);
        
        messageByteCount = 0;
      }
      else {

        if(messageByteCount == 4) this.CMD = inByte;
        else {
          data[messageByteCount - 5] = inByte;
        }
      }
    }
  }

  void send(int CMD) {
    if(this.serializableObjects.containsKey(CMD)) {
      
      int[] data = ((SerialSerializable)this.serializableObjects.get(CMD)).toSerialStreem(CMD);
      
      serial.write('>');
      serial.write('>');
      serial.write('>');
      serial.write((byte)CMD);
      
      for(int i = 0; i < data.length; i++){
        this.serial.write((byte)data[i]);
      }
      
      serial.write('*');
      serial.write('*');
      serial.write('*');
    }
  }

  void callReceiver(int CMD) {
    if(this.serializableObjects.containsKey(CMD)) {
      ((SerialSerializable)this.serializableObjects.get(CMD)).parseSerialStreem(this.CMD, this.data);
    }
  }
}
