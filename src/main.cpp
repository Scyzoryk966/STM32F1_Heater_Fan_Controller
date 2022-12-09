#include <Arduino.h>

#include <DHT_U.h>

#define MAX_MESSAGE_LENGTH 255

#define LED_PIN PB12
#define FAN_PWM PB7
#define BUTTON PB9
#define ANALOG_IN PA7

#define DHTPIN PB8

#define DHTTYPE    DHT22     // DHT 22 (AM2302)

//Define variables
//const uint8_t headerData[3] = {'<', '-', '-'};
const uint8_t headerData[3] = {'`','`','`'}; // change for testing purposes
bool enable = true;
char recivedData[255];
int pwmFanSpeed = 0; //0 to 65535 (PWM)
int tempResult = 0;
float actualTemp = 0;
int buttonValue = 0;
//DHT temperature and humidity sensor instence
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;
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
void DHTinfo();
float DHTGetTemp();
//---

void setup()
{
  //Set pin modes
  pinMode(LED_PIN, OUTPUT);
  pinMode(FAN_PWM, PWM);
  pinMode(BUTTON, INPUT_PULLDOWN);
  pinMode(ANALOG_IN, INPUT);
  //DHT init
  dht.begin();
  //serial init
  Serial.begin(115200); //serial baud
  Serial.setTimeout(10);
  delay(1000);
  Serial.print("Arduino READY\n");
  serialWriteHelp();
  //DHT info + set minimum delay time
  DHTinfo();
  //Init LED confirmation
  blinkLED(5, 100);
  digitalWrite(LED_PIN, HIGH);
}

void loop()
{
  pwmWrite(FAN_PWM, pwmFanSpeed);   
  
  if(buttonValue == LOW && digitalRead(BUTTON) == HIGH){      //Button click with positive edge - move to function
    Serial.println("Button clicked!\n");delay(10);    
  }
  buttonValue = digitalRead(BUTTON);

  if(asyncPeriodBool(delayMS)){
    actualTemp = DHTGetTemp();
    Serial.println(actualTemp);delay(10);
  }

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
  Serial.print("To send message via Serial port:\n"); delay(10);
  Serial.print("\t1. Write header: \"<--\" to serial buffer\n"); delay(10);
  Serial.print("\t2. Send date within 15 seconds window between characters\n"); delay(10);
  Serial.print("\t3. End data stream by sending \\n characters aka [ENTER]\n"); delay(10);
  Serial.print("Serial comands:\n"); delay(10);
  Serial.print("\t1. Send HEADER + \"H\" - brings up this message\n"); delay(10);
  Serial.print("\t2. Send HEADER + \"S + Speed\" [0-100] - sets fan speed\n\n"); delay(10);
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

void DHTinfo()
{
  Serial.print(F("DHTxx Unified Sensor Example\n"));
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.print(F("------------------------------------\n"));
  Serial.print(F("Temperature Sensor\n"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.print(F("------------------------------------\n"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.print(F("Humidity Sensor\n"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.print(F("------------------------------------\n"));
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;
}

float DHTGetTemp()
{
  // Delay between measurements.
  //delay(delayMS);
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!\n"));
  }
  else {
    //Serial.print(F("Temperature: "));
    //Serial.print(event.temperature);
    //Serial.println(F("°C"));
    return event.temperature;
  }
  // Get humidity event and print its value.
  /*
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
  }
  */
 return 0;
}