class MessageView implements SerialSerializable{
  int x,y,width,height,fontSize;
  
  int[] mType;
  String[] messages;
  
  public boolean isUpdated = false;
  
  MessageView(int x, int y, int width, int height, int fontSize){
    this.x = x;
    this.y = y;
    this.width = width;
    this.height = height;
    this.fontSize = fontSize;
    this.messages = new String[height / fontSize];
    this.mType = new int[height / fontSize];
    
    this.update();
  }
  
  void update(){
    stroke(255);
    fill(0);
    rect(this.x,this.y,this.width,this.height);
    
    textFont(font, this.fontSize);
    textAlign(LEFT,BOTTOM);
    for(int i = 0; i < this.mType.length; i++){
      if(this.messages[i] == null) continue;
      switch(this.mType[i]){
        case 1:
          fill(200,50,50);
          break;
        case 2:
          fill(255,0,0);
          break;
        default:
          fill(255);
      }
      text(this.messages[i], this.x + 2, this.y + this.fontSize * (i+1));
    }
    
    this.isUpdated = false;
  }
  
  int[] toSerialStreem(int CMD){return null;}
  
  void parseSerialStreem(int CMD, int[] data){
    String s = "";
    for(int i = 1; i < data.length; i++){
      if(data[i] == '*' && data[i+1] == '*') break;
      s += char(data[i]);
    }
    
    for(int i = this.messages.length - 1; i > 0; i--){
      this.messages[i] = this.messages[i-1];
      this.mType[i] = this.mType[i-1];
    }
    this.messages[0] = s;
    this.mType[0] = data[0];
    
    this.isUpdated = true;
  }
  
}
