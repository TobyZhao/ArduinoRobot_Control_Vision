double leftreadSerialCmd(){
  String inCmd = "";
  String inValue = "";
  bool subCmdPresent = false;
  short int valueCount = 0;
  float values[] = {0,0};
  if(Serial.available() > 0){
    while(Serial.available() > 0){
      int inChar = Serial.read();
      if(inChar==' ' || inChar=='\n'){
      }else if(isDigit(inChar)){
        inValue += (char)inChar;
      }else if( inChar == '.' || inChar == '-'){
        inValue += (char)inChar;
      }else if( inChar == ',' ){
        if (inValue != ""){
          values[valueCount] = inValue.toFloat();
        }
        inValue = "";
        valueCount++;
      }else if(isAscii(inChar)){
        inCmd = inCmd + (char)inChar;
      }

      if (inChar == '\n'){
        if (inValue != ""){
          values[valueCount] = inValue.toFloat();
        }
        inCmd.replace("\n","");
        inCmd.replace(" ","");

        if (inCmd == "Hi_Arduino" ){
          Serial.println("Hi_Raspberry");
        }else if (inCmd == "setValue"){
          return values[0];
        }else{
          //Serial.println("cmd_Error");
          //Serial.println("<"+inCmd+">");
        }
      }
    }
  }
}

