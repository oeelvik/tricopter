
void gsReceive(int inByte){
  gsMessageByteCount++;

  if(!gsInMessage  && inByte != (int)'>') { //Not this protocol
    gsMessageByteCount = 0;
  } 
  else if(inByte == (int)'>' && gsMessageByteCount == 3) { //prefix received
    gsInMessage = true;
    gsPostFixCount = 0;
    gsDataByteCount = 0;
    gsCMD = 0;
  } 
  else if (gsInMessage) {

    if(inByte == (int)'*') gsPostFixCount++;
    else gsPostFixCount = 0;
    
    if(gsPostFixCount == 3 || gsMessageByteCount - 5 >= GS_BYTE_LIMIT) { //All bytes in message reseived
      gsInMessage = false;
      gsMessageByteCount = 0;
      gsDataByteCount = gsDataByteCount - 2;
      gsCallReceiver();
    }
    else {

      if(gsMessageByteCount == 4) gsCMD = inByte;
      else {
        gsData[gsMessageByteCount - 5] = inByte;
        gsDataByteCount++;
      }
    }
  }
}

void gsCallReceiver(){
  switch(gsCMD){
    case 5: //request config
      TriGUIsendConfig();
      break;
    case 6: //setConfig
      if(gsDataByteCount == CV_END_BYTE + 1){
        TriGUIsendMessage(0, "Configuration received by tricopter");
        setConfig(gsData);
      } else TriGUIsendMessage(0, "Byte count Missmatch");
      break;
  }
}
