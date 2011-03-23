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
  
  int sensValue[30];
  int i=0;
  int maxValue = 50;
  char jSonString[10];
  char value[5];
  
  void setup() {  
      Serial.begin(9600);
      Serial.println("********************Testing aJson*****************\r\n");
      rcbUART.begin(9600);
      pinMode(CTS, OUTPUT);      //CTS
      pinMode(RTS, INPUT);       //RTS
      pinMode(LED, OUTPUT);
    }
  
  void loop() {
    if(digitalRead(RTS) == HIGH){
        digitalWrite(LED,HIGH);
        i=senden();
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
      digitalWrite(CTS,HIGH);
      delay(1000);
      Serial.println("Arrays angelegt...");
      rcbUART.print("{\"Temperatur\":[");
      for(i = 0; i<maxValue;i++){
          if(i==0){
              strcpy(jSonString,"0,");
          }else if (i==maxValue-1){
              sprintf(value,"%d", i*10);
              strcpy(jSonString, value);
          }else{
              sprintf(value,"%d", i);
              strcpy(jSonString, value);
              strcat(jSonString, ",");
          } 
        
          if(digitalRead(RTS) == HIGH){
              rcbUART.print(jSonString);
          }else{
              digitalWrite(CTS,LOW);
              return -1;
          }
       }
       
      rcbUART.print("],\"Zeit\":[");
      
      for(i = 0; i<maxValue;i++){
          if(i==0){
              strcpy(jSonString,"0,");
          }else if (i==maxValue-1){
              sprintf(value,"%d", i*10);
              strcpy(jSonString, value);
          }else{
              sprintf(value,"%d", i*10);
              strcpy(jSonString, value);
              strcat(jSonString, ",");  
          }
          if(digitalRead(RTS) == HIGH){
              rcbUART.print(jSonString);
          }else{
              digitalWrite(CTS,LOW);
              return -1;
          }
  }  
  rcbUART.print("]}");
  digitalWrite(CTS,LOW);
  Serial.println("Daten gesendet...");
  return 1;
 }
 
