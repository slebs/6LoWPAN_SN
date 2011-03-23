  #include <NewSoftSerial.h>  

  #define CTS 2
  #define RTS 3
  #define RX 4
  #define TX 5
  #define LED 13
  
  // Erstellen der Software UART auf den PINS RX und TX
  //  Es wird eine Software UART verwendet, da über die
  //  Hardware UART der Arduino programmiert wird. So wird eine 
  //  Beeinflussung durch den Arduino auf das RCB vorgebeugt
  NewSoftSerial rcbUART(RX, TX); //Rx, Tx
  

  int i=0;
  char jSonString[1400];
  char value[5];
  
  void setup() {     
//    Serial.begin(9600);
    rcbUART.begin(9600);
    pinMode(CTS, OUTPUT);      //CTS
    pinMode(RTS, INPUT);       //RTS
    pinMode(LED, OUTPUT);
    

  }
  
  void loop() {
    if(digitalRead(RTS) == HIGH){
      digitalWrite(LED,HIGH);
      Serial.println("........................................\n");
      senden();
      Serial.println("........................................\n");
      
      if(i>0){
        Serial.println("YIPPI!! - SENSORWERTE ÜBERTRAGEN :-D");
        digitalWrite(LED,LOW);
      }else{
        Serial.println("ERROR");
        digitalWrite(LED,LOW);
      }
    }
  }
  
 int senden(){
    digitalWrite(CTS, HIGH);
    delay(1000);
    
    if(digitalRead(RTS) == HIGH){
      
      rcbUART.println("<data>");
      rcbUART.println("<temp>");
      
      for(i = 0; i<360; i++){
        if(digitalRead(RTS) == HIGH){
          rcbUART.println(i);
        } else {
          digitalWrite(CTS, LOW);
          return -1;
        }
      }
      
      rcbUART.println("</temp>");
      rcbUART.println("</data>");
      digitalWrite(CTS, LOW);
      return 1;
    }
 }
 
