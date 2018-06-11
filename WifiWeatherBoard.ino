#include "SparkFunBME280.h"
#include "Wire.h"
#include <WiFi.h>
#include <driver/adc.h>

BME280 bme;

// Variables for wifi server setup 
const char* ssid     = "Prdelka";
const char* password = "zderadovabrno"; 

WiFiClient client;
const int httpPort = 80;
bool deviceConnected = false;
bool oldDeviceConnected = false;
float oldWindvalue = 0;

// Variables and constants used in calculating the windspeed.
volatile unsigned long timeSinceLastTick = 0;
volatile unsigned long lastTick = 0;

// Variables and constants used in tracking rainfall
#define S_IN_DAY   86400
#define S_IN_HR     3600
#define NO_RAIN_SAMPLES 2000
volatile long rainTickList[NO_RAIN_SAMPLES];
volatile int rainTickIndex = 0;
volatile int rainTicks = 0;
int rainLastDay = 0;
int rainLastHour = 0;
int rainLastHourStart = 0;
int rainLastDayStart = 0;
long secsClock = 0;

String windDir = "";
float windSpeed = 0.0;
String THINGS_API_KEY = "FG9WCE7AB34KQ6GT";

// Pin assignment definitions
#define WIND_SPD_PIN 14
#define RAIN_PIN     25
#define WIND_DIR_PIN 35
#define AIR_RST      4
#define AIR_WAKE     15
#define DONE_LED     5
#define UV_PIN       36

void setup() 
{
  delay(5);    // The CCS811 wants a brief delay after startup.
  Serial.begin(115200);
  Wire.begin(21,22);

  pinMode(UV_PIN, INPUT);
  
  pinMode(DONE_LED, OUTPUT);
  digitalWrite(DONE_LED, LOW);

  // Wind speed sensor setup. The windspeed is calculated according to the number
  //  of ticks per second. Timestamps are captured in the interrupt, and then converted
  //  into mph. 
  pinMode(WIND_SPD_PIN, INPUT);     // Wind speed sensor
  attachInterrupt(digitalPinToInterrupt(WIND_SPD_PIN), windTick, RISING);

  // Rain sesnor setup. Rainfall is tracked by ticks per second, and timestamps of
  //  ticks are tracked so rainfall can be "aged" (i.e., rain per hour, per day, etc)
  pinMode(RAIN_PIN, INPUT);     // Rain sensor
  attachInterrupt(digitalPinToInterrupt(RAIN_PIN), rainTick, RISING);
  // Zero out the timestamp array.
  for (int i = 0; i < NO_RAIN_SAMPLES; i++) rainTickList[i] = 0;

  // BME280 sensor setup - these are fairly conservative settings, suitable for
  //  most applications. For more information regarding the settings available
  //  for the BME280, see the example sketches in the BME280 library.
  bme.settings.commInterface = I2C_MODE;
  bme.settings.I2CAddress = 0x77;
  bme.settings.runMode = 3;
  bme.settings.tStandby = 0;
  bme.settings.filter = 0;
  bme.settings.tempOverSample = 1;
  bme.settings.pressOverSample = 1;
  bme.settings.humidOverSample = 1;
  bme.begin();

  // Connect to WiFi network
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Visible WiFi connected signal for when serial isn't connected
  digitalWrite(DONE_LED, HIGH);
}

void loop() 
{
  static unsigned long outLoopTimer = 0;
  static unsigned long wundergroundUpdateTimer = 0;
  static unsigned long clockTimer = 0;
  static unsigned long tempMSClock = 0;

  // Create a seconds clock based on the millis() count. We use this
  //  to track rainfall by the second. We've done this because the millis()
  //  count overflows eventually, in a way that makes tracking time stamps
  //  very difficult.
  tempMSClock += millis() - clockTimer;
  clockTimer = millis();
  while (tempMSClock >= 1000)
  {
    secsClock++;
    tempMSClock -= 1000;
  }

  // This is a once-per-second timer that calculates and prints off various
  //  values from the sensors attached to the system.
  if (millis() - outLoopTimer >= 2000)
  {
    outLoopTimer = millis();

    Serial.print("\nTimestamp: ");
    Serial.println(secsClock);

    // Windspeed calculation, in mph. timeSinceLastTick gets updated by an
    //  interrupt when ticks come in from the wind speed sensor.
    if (timeSinceLastTick != 0) windSpeed = 1000.0/timeSinceLastTick;
    Serial.print("Windspeed: ");
    if (timeSinceLastTick != 0){
    Serial.print(windSpeed*1.492*0.44704);
    }else{
      Serial.print("0");
    }
    Serial.println(" mph");

    // Update temperature. This also updates compensation values used to
    //  calculate other parameters.
    Serial.print("Temperature: ");
    Serial.print(bme.readTempC(), 2);
    Serial.println(" degrees C");

    // Display relative humidity.
    Serial.print("%RH: ");
    Serial.print(bme.readFloatHumidity(), 2);
    Serial.println(" %");

    // Display pressure.
    Serial.print("Pres: ");
    Serial.print(bme.readFloatPressure() * 0.0002953);
    Serial.println(" in");

    // Calculate the wind direction and display it as a string.
    Serial.print("Wind dir: ");
    windDirCalc(analogRead(WIND_DIR_PIN));
    Serial.print("  ");
    Serial.println(windDir);

    // Calculate and display rainfall totals.
    Serial.print("Rainfall last hour: ");
    Serial.println(float(rainLastHour)*0.011, 3);
    Serial.print("Rainfall last day: ");
    Serial.println(float(rainLastDay)*0.011, 3);
    Serial.print("Rainfall to date: ");
    Serial.println(float(rainTicks)*0.011, 3);

    Serial.print("Luminous flux: ");
    int uvLevel = readLightSensor();//averageAnalogRead(UV_PIN);
    float outputVoltage = 3.3 * uvLevel/4095;
    float uvIntensity = mapfloat(outputVoltage, 0.99, 2.9, 0.0, 15.0); //Convert the voltage to a UV intensity level
    Serial.print(uvLevel);
    Serial.println("mW/cm^2");

    // Calculate the amount of rain in the last day and hour.
    rainLastHour = 0;
    rainLastDay = 0;
    // If there are any captured rain sensor ticks...
    if (rainTicks > 0)
    {
      // Start at the end of the list. rainTickIndex will always be one greater
      //  than the number of captured samples.
      int i = rainTickIndex-1;

      // Iterate over the list and count up the number of samples that have been
      //  captured with time stamps in the last hour.
      while ((rainTickList[i] >= secsClock - S_IN_HR) && rainTickList[i] != 0)
      {
        i--;
        if (i < 0) i = NO_RAIN_SAMPLES-1;
        rainLastHour++;
      }

      // Repeat the process, this time over days.
      i = rainTickIndex-1;
      while ((rainTickList[i] >= secsClock - S_IN_DAY) && rainTickList[i] != 0)
      {
        i--;
        if (i < 0) i = NO_RAIN_SAMPLES-1;
        rainLastDay++;
      }
      rainLastDayStart = i;
    }
  }


}

// Keep track of when the last tick came in on the wind sensor.
void windTick(void)
{
  timeSinceLastTick = millis() - lastTick;
  lastTick = millis();
}

// Capture timestamp of when the rain sensor got tripped.
void rainTick(void)
{
  rainTickList[rainTickIndex++] = secsClock;
  if (rainTickIndex == NO_RAIN_SAMPLES) rainTickIndex = 0;
  rainTicks++;
}

// For the purposes of this calculation, 0deg is when the wind vane
//  is pointed at the anemometer. The angle increases in a clockwise
//  manner from there.
void windDirCalc(int vin)
{
  if      (vin < 150) windDir="202.5";
  else if (vin < 300) windDir = "180";
  else if (vin < 400) windDir = "247.5";
  else if (vin < 600) windDir = "225";
  else if (vin < 900) windDir = "292.5";
  else if (vin < 1100) windDir = "270";
  else if (vin < 1500) windDir = "112.5";
  else if (vin < 1700) windDir = "135";
  else if (vin < 2250) windDir = "337.5";
  else if (vin < 2350) windDir = "315";
  else if (vin < 2700) windDir = "67.5";
  else if (vin < 3000) windDir = "90";
  else if (vin < 3200) windDir = "22.5";
  else if (vin < 3400) windDir = "45";
  else if (vin < 4000) windDir = "0";
  else windDir = "0";
}

//Takes an average of readings on a given pin
//Returns the average
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 

  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return(runningValue);  
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int readLightSensor(){
  adc1_config_width(ADC_WIDTH_BIT_10);   //Range 0-1023 
  adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_11);  //ADC_ATTEN_DB_11 = 0-3,6V
  return adc1_get_raw( ADC1_CHANNEL_0 ); //Read analog
}

void sendDataToThingspeak(){
  if (client.connect("api.thingspeak.com",80)) {
    String message = THINGS_API_KEY;
    message +="&field1=";
    message += String(cas);
    message +="&field2=";
    message += String(analog);
    message += "\r\n\r\n";
    client.print("POST /update HTTP/1.1\n");
    client.print("Host: api.thingspeak.com\n");
    client.print("Connection: close\n");
    client.print("X-THINGSPEAKAPIKEY: "+THINGS_API_KEY+"\n");
    client.print("Content-Type: application/x-www-form-urlencoded\n");
    client.print("Content-Length: ");
    client.print(message.length());
    client.print("\n\n");
    client.print(message);
  }
  client.stop();
 }

