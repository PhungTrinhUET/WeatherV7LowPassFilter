#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <DHT.h>
#include <SoftwareSerial.h>
#include <PubSubClient.h>

//=============================================***===========================================
//-----------------------------------------Temp + Humi---------------------------------------
#define DHT11PIN 4 // GPIO4
DHT dht(DHT11PIN, DHT11);

//=============================================***===========================================
//---------------------------------------BME280-PRESSURE-------------------------------------
Adafruit_BME280 bme;

//=============================================***===========================================
//-----------------------------------------Light Sensor--------------------------------------  
BH1750 bh1750;

//=============================================***===========================================
//--------------------------------------Wifi and PubSubClient--------------------------------
#include <WiFi.h>
const char *ssid = "Fatlab";
const char *password = "12345678@!";
const char *mqttServer = "192.168.1.148";
const int mqttPort = 1884;
WiFiClient espClient;
PubSubClient client(espClient);

//=============================================***===========================================
//----------------------------------- Weather Board Connection ------------------------------
SoftwareSerial mySerial(16, 17); // RX, TX
char databuffer[35];
double temp;

//=============================================***===========================================
//---------------------------------------- FAW counter---------------------------------------
volatile int IRQcount;
int pin = 33;
int pin_irq = 33; // IRQ that matches to pin 33
void IRQcounter()
{
  detachInterrupt(pin_irq);
  IRQcount++;
  int n = 1000;
  while (n > 0)
  {
    n--;
  }
}

//=============================================***===========================================
//========================================Void getBuffer=====================================
void getBuffer()
{
  int index;
  for (index = 0; index < 35; index++)
  {
    if (mySerial.available())
    {
      databuffer[index] = mySerial.read();
      if (databuffer[0] != 'c')
      {
        index = -1;
      }
    }
    else
    {
      index--;
    }
  }
}

int transCharToInt(char *_buffer, int _start, int _stop)
{
  int _index;
  int result = 0;
  int num = _stop - _start + 1;
  int _temp[num];
  for (_index = _start; _index <= _stop; _index++)
  {
    _temp[_index - _start] = _buffer[_index] - '0';
    result = 10 * result + _temp[_index - _start];
  }
  return result;
}

int transCharToInt_T(char *_buffer)
{
  int result = 0;
  if (_buffer[13] == '-')
  {
    result = 0 - (((_buffer[14] - '0') * 10) + (_buffer[15] - '0'));
  }
  else
  {
    result = ((_buffer[13] - '0') * 100) + ((_buffer[14] - '0') * 10) + (_buffer[15] - '0');
  }
  return result;
}

int WindDirection()
{
  return transCharToInt(databuffer, 1, 3);
}

float WindSpeedAverage()
{
  temp = 0.44704 * transCharToInt(databuffer, 5, 7);
  return temp;
}

float WindSpeedMax()
{
  temp = 0.44704 * transCharToInt(databuffer, 9, 11);
  return temp;
}

float RainfallOneHour()
{
  temp = transCharToInt(databuffer, 17, 19) * 25.40 * 0.01;
  return temp;
}

float RainfallOneDay()
{
  temp = transCharToInt(databuffer, 21, 23) * 25.40 * 0.01;
  return temp;
}

void setup()
{
  Serial.begin(115200);
  mySerial.begin(9600);

  // Connect to Wifi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Connect to MQTT Broker
  client.setServer(mqttServer, mqttPort);
  while (!client.connected())
  {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32Client"))
    {
      Serial.println("Connected to MQTT");
    }
    else
    {
      Serial.print("Failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }

  // DHT11 Setup
  Serial.begin(9600);
  dht.begin();

  // BME280 Setup
  Serial.println(F("BME280 test"));
  bool status;
  status = bme.begin(0x76);

  // BH1750 Setup
  bh1750.begin();
  Wire.begin();

  // Attach interrupt for FAW counter
  attachInterrupt(digitalPinToInterrupt(pin_irq), IRQcounter, FALLING);
}

//=============================================***===========================================
//-----------------------------------------Void Loop-----------------------------------------
void loop()
{
  ReadWeatherStation();
  CountingFAW();
  SendWeatherDataToBroker();
  delay(1000);
}

//=============================================***===========================================
//-----------------------------------Void ReadWeatherStation---------------------------------
void ReadWeatherStation()
{
  getBuffer();

  // DHT11
  float temperatureDHT = dht.readTemperature();
  float humidityDHT = dht.readHumidity();

  // BME280
  float pressureBME = bme.readPressure() / 100.0F;

  // BH1750
  float luxBH1750 = bh1750.readLightLevel();

  Serial.print("DHT11 - Temperature: ");
  Serial.print(temperatureDHT);
  Serial.print(" °C, Humidity: ");
  Serial.print(humidityDHT);
  Serial.println(" %");

  Serial.print("BME280 - Pressure: ");
  Serial.print(pressureBME);
  Serial.println(" hPa");

  Serial.print("BH1750 - Light: ");
  Serial.print(luxBH1750);
  Serial.println(" lux");
}

//=============================================***===========================================
//---------------------------------------Void CountingFaw------------------------------------
void CountingFAW()
{
  // Existing FAW counting logic here
}

//=============================================***===========================================
//-------------------------------Void SendWeatherDataToBroker--------------------------------
void SendWeatherDataToBroker()
{
  getBuffer();
  float windDirection = WindDirection();
  float windSpeedAverage = WindSpeedAverage();
  float windSpeedMax = WindSpeedMax();
  float rainfallOneHour = RainfallOneHour();
  float rainfallOneDay = RainfallOneDay();

  float temperatureDHT = dht.readTemperature();
  float humidityDHT = dht.readHumidity();
  float pressureBME = bme.readPressure() / 100.0F;
  float luxBH1750 = bh1750.readLightLevel();

  // Send data to individual MQTT topics
  client.publish("esp32/weatherstation/temperature", String(temperatureDHT).c_str());
  client.publish("esp32/weatherstation/humidity", String(humidityDHT).c_str());
  client.publish("esp32/weatherstation/pressure", String(pressureBME).c_str());
  client.publish("esp32/weatherstation/light", String(luxBH1750).c_str());
  client.publish("esp32/weatherstation/wind_direction", String(windDirection).c_str());
  client.publish("esp32/weatherstation/wind_speed_average", String(windSpeedAverage).c_str());
  client.publish("esp32/weatherstation/wind_speed_max", String(windSpeedMax).c_str());
  client.publish("esp32/weatherstation/rainfall_one_hour", String(rainfallOneHour).c_str());
  client.publish("esp32/weatherstation/rainfall_one_day", String(rainfallOneDay).c_str());
  client.publish("esp32/weatherstation/IRQcount", String(IRQcount).c_str());
}
