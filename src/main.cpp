#include <Arduino.h>
#define MAX_MESSAGE_LENGTH 255

#define LED_PIN PB12

//Define variables
const uint8_t headerData[3] = {'<', '-', '-'};
bool enable = true;
char testData[255];
//period related variables
unsigned long currTime = millis();
unsigned long prevTime = 0;
//Define functions
void reciveSerialData(bool *enable, char *recivedData);
bool waitForUserInputTimeout(bool *enable);
void serialWriteHelp();
bool asyncPeriodBool(unsigned long period);
void blinkLED(byte numBlinks, int onOffTime);
//---

void setup()
{
  //Set pin modes
  pinMode(LED_PIN, OUTPUT);
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
  reciveSerialData(&enable, testData);
  if (testData[0] == 'H' || testData[0] == 'h')
  {
    serialWriteHelp();
    //XDD
  }
  if (testData[0] != 0)
    Serial.println(testData);
  memset(testData, 0, sizeof(*testData));
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

void serialWriteHelp()
{
  Serial.print("To send message via Serial port:\n");
  Serial.print("\t1. Write header: \"<--\" to serial buffer\n");
  Serial.print("\t2. Send date within 15 seconds window between characters\n");
  Serial.print("\t3. End data stream by sending \\n characters aka [ENTER]\n");
  Serial.print("Serial comands:\n");
  Serial.print("\t1. Send HEADER + \"H\" - brings up this message\n");
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