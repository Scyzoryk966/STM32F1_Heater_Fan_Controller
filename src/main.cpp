#include <Arduino.h>
#define MAX_MESSAGE_LENGTH 255

#define LED_PIN PB12
#define FAN_PWM PB7
#define ANALOG_IN PA7

//Define variables
//const uint8_t headerData[3] = {'<', '-', '-'};
const uint8_t headerData[3] = {'`'}; // change for testing purposes
bool enable = true;
char recivedData[255];
int pwmFanSpeed = 65535; //0 to 65535 (PWM)
int tempResult = 0;
//period related variables
unsigned long currTime = millis();
unsigned long prevTime = 0;
//Define functions
void reciveSerialData(bool *enable, char *recivedData);
bool waitForUserInputTimeout(bool *enable);
void changeFanSpeed(int speed);
void serialWriteHelp();
bool asyncPeriodBool(unsigned long period);
void blinkLED(byte numBlinks, int onOffTime);
//---

void setup()
{
  //Set pin modes
  pinMode(LED_PIN, OUTPUT);
  pinMode(FAN_PWM, PWM);
  pinMode(ANALOG_IN, INPUT);
  //serial init
  Serial.begin(115200); //serial baud
  Serial.setTimeout(10);
  delay(1000);
  Serial.print("Arduino READY\n");
  serialWriteHelp();
  blinkLED(5, 100);
  digitalWrite(LED_PIN, HIGH);
}

void loop()
{
  pwmWrite(FAN_PWM, pwmFanSpeed);        

  reciveSerialData(&enable, recivedData);
  if (recivedData[0] == 'H' || recivedData[0] == 'h')
  {
    serialWriteHelp();
  } 
  else if (recivedData[0] == 'S' || recivedData[0] == 's')
  {
    changeFanSpeed(atoi(recivedData + 1)); //recivedData + 1 = same string witout first character
  }
  
  if (recivedData[0] != 0)
    Serial.println(recivedData);
  memset(recivedData, 0, sizeof(*recivedData));
}

void reciveSerialData(bool *enable, char *recivedData)
{
  if (Serial.available() > 0 && *enable)
  {
    uint8_t b = Serial.read();
    bool similarToHeader = false;
    if (b == headerData[0]) // compare to header
    {
      similarToHeader = true;
      for (unsigned int i = 1; similarToHeader && (i < sizeof(headerData)); i++)
      {
        if (!waitForUserInputTimeout(enable))
        {
          memset(recivedData, 0, sizeof(*recivedData));
          return;
        }
        b = Serial.read();
        if (b != headerData[i])
        {
          similarToHeader = false; // jeżeli któryś bit nie zgadza się z nagłowkiem warunek nie zostaje spełniony
        }
      }
    }

    if (similarToHeader)
    {
      Serial.println("Header found!!!");
      int dataCounter = 0;
      do
      {
        if (!waitForUserInputTimeout(enable))
        {
          memset(recivedData, 0, sizeof(*recivedData));
          return;
        }
        b = Serial.read();
        recivedData[dataCounter] = b;
        dataCounter++;
        Serial.print((char)b);
      } while (b != '\n');
    }
    else
    {
      Serial.println("Header not recognized...Send \"<--\" + \"H\" for help.\n");
      Serial.read();
      return;
    }
  }
}

bool waitForUserInputTimeout(bool *enable)
{
  int timeout = 1;
  prevTime = millis();
  while (Serial.available() == 0 && *enable) //w8 for serial data
  {
    if (asyncPeriodBool(5000))
    {
      Serial.print("Wait for user input... Retry:");
      Serial.println(timeout);
      timeout++;
    }
    if (timeout > 3)
    {
      Serial.println("Data input timeout...");
      return false;
    }
  }
  return true;
}

void changeFanSpeed(int speed)
{
  int tempSpeed = speed;
  if(speed > 100)
    tempSpeed = 100;
  else if(speed < 0)
    tempSpeed = 0;
  Serial.print("Speed was changed to: ");
  Serial.print(speed, DEC);
  Serial.print("%.\n");
  pwmFanSpeed = map(tempSpeed, 0, 100, 0, 65535);
}

void serialWriteHelp()
{
  Serial.print("To send message via Serial port:\n");
  Serial.print("\t1. Write header: \"<--\" to serial buffer\n");
  Serial.print("\t2. Send date within 15 seconds window between characters\n");
  Serial.print("\t3. End data stream by sending \\n characters aka [ENTER]\n");
  Serial.print("Serial comands:\n");
  Serial.print("\t1. Send HEADER + \"H\" - brings up this message\n");
  Serial.print("\t2. Send HEADER + \"S + Speed\" [0-100] - sets fan speed\n");
}

bool asyncPeriodBool(unsigned long period)
{
  currTime = millis();
  if (currTime - prevTime >= period)
  {
    prevTime = currTime;
    return true;
  }
  else
  {
    return false;
  }
}

void blinkLED(byte numBlinks, int onOffTime)
{ //classy Blink with delay
  for (byte n = 0; n < numBlinks; n++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(onOffTime);
    digitalWrite(LED_PIN, LOW);
    delay(onOffTime);
  }
}