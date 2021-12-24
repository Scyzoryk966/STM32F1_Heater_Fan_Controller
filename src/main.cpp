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
  Serial.setTimeout(10);
  delay(100);
  Serial.print("Arduino Gotowe1\n");
  blinkLED(10,50);
}

void loop() {
  blinkLED(1,100);

}

// char* ReciveSerialMessage(){
//   static char message[MAX_MESSAGE_LENGTH];
//   static unsigned int message_pos = 0;
//   while (Serial.available() > 0)
//  {
//    char inByte = Serial.read();
//    if ( inByte != '\n' && (message_pos < MAX_MESSAGE_LENGTH - 1) )
//    {
//      message[message_pos] = inByte;
//      message_pos++;
//    }
//    else
//    {
//      message[message_pos] = '\0';
//      message_pos = 0;
//      return message;
//    }
//  }
//  return 0;
// }

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