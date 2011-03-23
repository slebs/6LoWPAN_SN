  #include <NewSoftSerial.h>
  
  NewSoftSerial rcbUART(4, 5); //Rx, Tx
  
  
  void setup()
  {
    rcbUART.begin(115200);
    Serial.begin(115200);
    pinMode(13, OUTPUT);
  }
  
  void loop()
  {
      if (rcbUART.available())
      {
        //digitalWrite(13,HIGH);
        Serial.print(rcbUART.read(),BYTE);
      }
      
      if (Serial.available()){
        //digitalWrite(13,HIGH);
        rcbUART.print(Serial.read(),BYTE);
      }
      //digitalWrite(13,LOW);
  }
  
