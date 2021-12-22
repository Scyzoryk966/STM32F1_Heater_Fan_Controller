#include <Arduino.h>
#define MAX_MESSAGE_LENGTH 255

#define LED_PIN PB12

//Definicja funkcji
char * ReciveSerialMessage();
void blinkLED(byte numBlinks, int onOffTime);
//---

void setup() {
  //Set pin modes
  pinMode(LED_PIN, OUTPUT);
  //serial init
  Serial.begin(9600);  //ustawiamy przepustowość łącza szeregowego
  Serial.setTimeout(10); //ustawiamy czas utracenia połączaniaSERIAL, jest bardzo niski bo nie działamy synchronicznie
  delay(100);            //migamy ledem na płytce sygnalizując że płytka się uruchomiła
  Serial.print("Arduino Gotowe\n");
  delay(100);
}

void loop() {
 //Check to see if anything is available in the serial receive buffer
 
}

char * ReciveSerialMessage(){
  while (Serial.available() > 0)
 {
   static char message[MAX_MESSAGE_LENGTH];
   static unsigned int message_pos = 0;
   char inByte = Serial.read();
   if ( inByte != '\n' && (message_pos < MAX_MESSAGE_LENGTH - 1) )
   {
     message[message_pos] = inByte;
     message_pos++;
   }
   else
   {
     message[message_pos] = '\0';
     return message;
     message_pos = 0;
   }
 }
}

void blinkLED(byte numBlinks, int onOffTime)
{ //funkcja miganie leda na płytce - do debugowania
  for (byte n = 0; n < numBlinks; n++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(onOffTime);
    digitalWrite(LED_PIN, LOW);
    delay(onOffTime);
  }
}