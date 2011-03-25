  #include <NewSoftSerial.h>
  
  NewSoftSerial rcbUART(4, 5); //Rx, Tx
  
  
  void setup()
  {
    rcbUART.begin(9600);
    Serial.begin(9600);
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
  
