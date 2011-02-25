interface SerialSerializable {
  int[] toSerialStreem();
  void parseSerialStreem(int[] data);
}

class SerialComHandler {
  final int BYTE_LIMIT = 50;

  HashMap serializableObjects = new HashMap();

  int CMD;
  int[] data = new int[BYTE_LIMIT];

  boolean inMessage = false;
  int messageByteCount = 0;
  int postFixCount = 0;

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
      }

      if(messageByteCount == 4) this.CMD = inByte;
      else data[messageByteCount - 5] = inByte;
    }
  }

  void send(int CMD) {
    if(this.serializableObjects.containsKey(CMD)) {
      //TODO: Send by serial
      //((SerialSerializable)this.serializableObjects.get(CMD)).parseSerialStreem(this.data);
    }
  }

  void callReceiver(int CMD) {
    if(this.serializableObjects.containsKey(CMD)) {
      ((SerialSerializable)this.serializableObjects.get(CMD)).parseSerialStreem(this.data);
    }
  }
}
