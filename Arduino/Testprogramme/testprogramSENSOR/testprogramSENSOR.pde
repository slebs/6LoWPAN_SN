  #include <NewSoftSerial.h>
  //#include "aJSON.h"
  #include <avr/pgmspace.h>
  

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
//    freeMem("start");    
    Serial.begin(9600);
    Serial.println("********************\nTesting aJson\n*****************\n");
    rcbUART.begin(9600);
    pinMode(CTS, OUTPUT);      //CTS
    pinMode(RTS, INPUT);       //RTS
    pinMode(LED, OUTPUT);
    

  }
  
  void loop() {
//    if(digitalRead(RTS) == HIGH){
//      freeMem("start");
      digitalWrite(LED,HIGH);
      Serial.println("........................................\n");
      senden();
      delay(5000);
      Serial.println("........................................\n");
//      if(werte>0){
//        Serial.println("YIPPI!! - SENSORWERTE ÜBERTRAGEN :-D");
//        digitalWrite(LED,LOW);
//      }else{
//        Serial.println("ERROR");
//        digitalWrite(LED,LOW);
//      }
//    }
  }
  
 int senden(){

  Serial.println("Arrays angelegt...");
      Serial.print("{\"Temperatur\":[");
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
              Serial.print(jSonString);
       }
       
      Serial.print("],\"Zeit\":[");
      
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
              Serial.print(jSonString);
  }  
  Serial.print("]}");
  Serial.println("Daten gesendet...");

/*
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
*/
  return 1;
 }
 
//Code to print out the free memory

struct __freelist {
  size_t sz;
  struct __freelist *nx;
};

extern char * const __brkval;
extern struct __freelist *__flp;

uint16_t freeMem(uint16_t *biggest)
{
  char *brkval;
  char *cp;
  unsigned freeSpace;
  struct __freelist *fp1, *fp2;

  brkval = __brkval;
  if (brkval == 0) {
    brkval = __malloc_heap_start;
  }
  cp = __malloc_heap_end;
  if (cp == 0) {
    cp = ((char *)AVR_STACK_POINTER_REG) - __malloc_margin;
  }
  if (cp <= brkval) return 0;

  freeSpace = cp - brkval;

  for (*biggest = 0, fp1 = __flp, fp2 = 0;
     fp1;
     fp2 = fp1, fp1 = fp1->nx) {
      if (fp1->sz > *biggest) *biggest = fp1->sz;
    freeSpace += fp1->sz;
  }

  return freeSpace;
}

uint16_t biggest;

void freeMem(char* message) {
  Serial.print(message);
  Serial.print(":\t");
  Serial.println(freeMem(&biggest));
}

