#include <Arduino.h>
#include <EEPROM.h>
#include <DHT_U.h>

#define MAX_MESSAGE_LENGTH 255

#define LED_PIN PB12
#define RELAY_PIN PB0
#define FAN_PWM PB6
#define BUTTON PB9
#define ANALOG_IN PA7

#define DHTPIN  PB11
#define DHTTYPE DHT22     // DHT 22 (AM2302)

//Define variables
//const uint8_t headerData[3] = {'<', '-', '-'};
const uint8_t headerData[3] = {'`','`','`'}; // change for testing purposes
bool enable = true;
char recivedData[255];
int pwmFanSpeed = 65535; //0 to 65535 (PWM)
int tempResult = 0;
int buttonValue = 0;
//Change fan speed variables
bool changeFanSpeedFlag = false;
byte savedFanSpeed = EEPROM.read(0);
int actualFanSpeed = savedFanSpeed;
int lastFanSpeed = 0;
bool dataSaveFlag = false;
//DHT temperature and humidity sensor instence
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;
float actualTemp = 0;
float setpointTemp = 0;
//period related variables
//unsigned long currTime = millis();
unsigned long prevTimeSerial = 0;
unsigned long prevTimeDHT = 0;
unsigned long prevTimeEEPROM = 0;
//Define functions
void reciveSerialData(bool *enable, char *recivedData);
bool waitForUserInputTimeout(bool *enable);
int changeFanSpeed(int speed);
void buttonSpeedChangeHandler();
void serialWriteHelp();
bool asyncPeriodBool(unsigned long period, unsigned long *prevTime);
void blinkLED(byte numBlinks, int onOffTime);
void DHTinit();
float DHTGetTemp();
//-----------------------------------------
void setup()
{
  //Set pin modes
  pinMode(LED_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(FAN_PWM, PWM);
  pinMode(BUTTON, INPUT_PULLDOWN);
  pinMode(ANALOG_IN, INPUT);
  //serial init
  Serial.begin(115200); //serial baud
  Serial.setTimeout(10);
  delay(1000);
  Serial.print("Arduino READY\n");
  serialWriteHelp();
  //DHT info + set minimum delay time
  DHTinit();
  //Init LED confirmation
  blinkLED(5, 100);
  changeFanSpeed(actualFanSpeed);
  setpointTemp = EEPROM.read(2) + (float)EEPROM.read(3)/100;
  Serial.println(setpointTemp);
}

void loop()
{
  //Write PWM fan speed--------------------------------
  
  pwmWrite(FAN_PWM, pwmFanSpeed);   
  
  //Change speed via button, EEPROM speed save------------------------
  
  buttonSpeedChangeHandler();
  
  //Assign actual temperature--------------------------
  
  if(asyncPeriodBool(delayMS, &prevTimeDHT)){
    actualTemp = DHTGetTemp();
    //Serial.println(actualTemp);delay(10);
  }

  //Assign actual temperature--------------------------
  
  if(actualTemp >= setpointTemp)
  {
    digitalWrite(RELAY_PIN, HIGH);
  }
  else
  {
    digitalWrite(RELAY_PIN, LOW);
  }

  //Serial data readout--------------------------------
  reciveSerialData(&enable, recivedData);
  if (recivedData[0] == 'H' || recivedData[0] == 'h')
  {
    serialWriteHelp();
  } 
  else if (recivedData[0] == 'S' || recivedData[0] == 's')
  {
    changeFanSpeed(atoi(recivedData + 1)); //recivedData + 1 = same string witout first character
  }
  else if (recivedData[0] == 'T' || recivedData[0] == 't')
  {
    if (recivedData[1] == 'C' || recivedData[1] == 'c')
    {
      Serial.println(setpointTemp = atof(recivedData + 2));
      EEPROM.write(2, (int)setpointTemp);
      EEPROM.write(3, ((int)((setpointTemp - (int)setpointTemp)*100)));
      Serial.print("Ustawiona temperatura:");Serial.println(setpointTemp); delay(10); 
    }
    if (recivedData[1] == 'R' || recivedData[1] == 'R')
    {
      Serial.print(EEPROM.read(2));
      Serial.print(".");
      Serial.println(EEPROM.read(3));
    }
    else
    {
      Serial.print(actualTemp); Serial.print(" °C\n"); delay(10); // print actual temperature
    }
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
      Serial.println("Header not recognized...Send \"```\" + \"H\" for help.\n");
      Serial.read();
      return;
    }
  }
}

bool waitForUserInputTimeout(bool *enable)
{
  int timeout = 1;
  prevTimeSerial = millis();
  while (Serial.available() == 0 && *enable) //w8 for serial data
  {
    if (asyncPeriodBool(5000, &prevTimeSerial))
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

int changeFanSpeed(int speed)
{
  int tempSpeed = speed;
  if(speed > 100)
    tempSpeed = 0;
  else if(speed < 0)
    tempSpeed = 100;
  // Serial.print("Speed was changed to: ");
  // Serial.print(tempSpeed, DEC);
  // Serial.print("%.\n");
  pwmFanSpeed = map(tempSpeed, 0, 100, 0, 65535);
  return tempSpeed;
}

void buttonSpeedChangeHandler()
{
  //Speed change after button click and seting it to 100%-
  if(changeFanSpeedFlag){
    actualFanSpeed = changeFanSpeed(actualFanSpeed + 10);
    if(actualFanSpeed < 100 && actualFanSpeed != 50)
      blinkLED(1,200);
    if(actualFanSpeed == 50)
      blinkLED(2,100);
    if(actualFanSpeed == 100)
      blinkLED(3,66);
    Serial.print("Speed was changed to: ");
    Serial.print(actualFanSpeed, DEC);
    Serial.print("%.\n");
    changeFanSpeedFlag = false;
    dataSaveFlag = true;
  }
  //Button click with positive edge---------------------
  if(buttonValue == LOW && digitalRead(BUTTON) == HIGH && !changeFanSpeedFlag){
    changeFanSpeed(100);      //Set speed to 100 for few miliseconds to get fan into speed
    delay(100);
    changeFanSpeedFlag = true;      //flag to change speed of fan from 100 to proper speed
  }
  buttonValue = digitalRead(BUTTON);
  
  //Save data after 20s aftes change speed-------------
  
  if(dataSaveFlag && actualFanSpeed != savedFanSpeed){
    if(asyncPeriodBool(20000, &prevTimeEEPROM)){
      //EEPROM.update(0, actualFanSpeed);   //-----------------------------------------------------Uncomment to activate EEPROM speed save
      Serial.println("DataSaved\n");delay(10);
      dataSaveFlag = false;
    }
  }
  else
  {
    prevTimeEEPROM = millis();
  }

}

void serialWriteHelp()
{
  Serial.print("To send message via Serial port:\n"); delay(10);
  Serial.print("\t1. Write header: \"```\" (3 x tylda key) to serial buffer.\n"); delay(10);
  Serial.print("\t2. Send date within 15 seconds window between characters.\n"); delay(10);
  Serial.print("\t3. End data stream by sending \\n characters aka [ENTER].\n"); delay(10);
  Serial.print("Serial comands:\n"); delay(10);
  Serial.print("\t1. Send HEADER + \"H\" - brings up this message.\n"); delay(10);
  Serial.print("\t2. Send HEADER + \"S + Speed\" [0-100] - sets fan speed.\n"); delay(10);
  Serial.print("\t3. Send HEADER + \"T\" - print actual temperature value in °C.\n"); delay(10);
  Serial.print("\t4. Send HEADER + \"TC + value of temp\" [0.00-50.00°C] - sets value of temperature setpoint.\n"); delay(10);
  Serial.print("\t4. Send HEADER + \"TR\" read value of temperature setpoint.\n"); delay(10);
}

bool asyncPeriodBool(unsigned long period, unsigned long *prevTime)
{
  unsigned long currTime = millis();
  if (currTime - *prevTime >= period)
  {
    *prevTime = currTime;
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

void DHTinit()
{
  //DHT init
  dht.begin();
  delay(1000);

  Serial.print(F("DHTxx Unified Sensor Example\n"));
  // Print temperature sensor details.
  sensor_t sensor;
  delay(100);
  dht.temperature().getSensor(&sensor);
  Serial.print(F("------------------------------------\n"));
  Serial.print(F("Temperature Sensor\n"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.print(F("------------------------------------\n"));delay(10);
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.print(F("Humidity Sensor\n"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.print(F("------------------------------------\n"));delay(10);
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