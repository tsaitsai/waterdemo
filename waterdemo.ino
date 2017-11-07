/***************************************************
  Eric Tsai 2017-11-6
  Combined Adafruit's Temperature sensor code with 
  digital compass code.

  Tutorial & bulk of code:
  https://learn.adafruit.com/adafruit-io-basics-temperature-and-humidity/overview

  Wiring & Hardware:
  ESP8266 microcontroller
  HMC5883L digital compass
  
    "D2" = SDA from compass
    "D1" = SCL from compass
    vcc for compass is 3.3V
    gnd
 ****************************************************/

// Libraries
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
//#include "DHT.h"
#include <Wire.h> //I2C Arduino Library, for magnet sensor
#define addr 0x1E //I2C Address for The HMC5883
// VCC = 3.3V
// GND = GND
// SDA = A4  for EsP8266, D2 = GPIO 4
// SCL = A5  for esp8266, D1 = GPIO 5


// DHT 11 sensor
//#define DHTPIN 5
//#define DHTTYPE DHT22 

// WiFi parameters
#define WLAN_SSID       "XXXXXXXXX"   //tsai:  change this to your wifi network SSID
#define WLAN_PASS       "XXXXXXXXX"     //tsai:  change this to your wifi network password

// Adafruit IO
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "waterdemo"
#define AIO_KEY         "59c87f5c89fe4bc2b3ad97ca0c4bef24"

// DHT sensor
//DHT dht(DHTPIN, DHTTYPE, 15);

// Functions
void connect();

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;

// Store the MQTT server, client ID, username, and password in flash memory.
const char MQTT_SERVER[] = AIO_SERVER;

// Set a unique MQTT client ID using the AIO key + the date and time the sketch
// was compiled (so this should be unique across multiple devices for a user,
// alternatively you can manually set this to a GUID or other random value).
const char MQTT_CLIENTID[] = AIO_KEY __DATE__ __TIME__;
const char MQTT_USERNAME[] = AIO_USERNAME;
const char MQTT_PASSWORD[] = AIO_KEY;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, AIO_SERVERPORT, MQTT_CLIENTID, MQTT_USERNAME, MQTT_PASSWORD);/****************************** Feeds ***************************************/

// Setup feeds for temperature & humidity
//const char TEMPERATURE_FEED[] PROGMEM = AIO_USERNAME "/feeds/temperature";
//Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, TEMPERATURE_FEED);
const char MAG_FIELD_FEED[] = AIO_USERNAME "/feeds/magnetometer";
Adafruit_MQTT_Publish mq_mag_field = Adafruit_MQTT_Publish(&mqtt, MAG_FIELD_FEED);

//const char HUMIDITY_FEED[] PROGMEM = AIO_USERNAME "/feeds/humidity";
//Adafruit_MQTT_Publish humidity = Adafruit_MQTT_Publish(&mqtt, HUMIDITY_FEED);
const char MAG_RATE_FEED[] = AIO_USERNAME "/feeds/rate";
Adafruit_MQTT_Publish mq_mag_rate = Adafruit_MQTT_Publish(&mqtt, MAG_RATE_FEED);

const char MAG_TOTAL_FEED[] = AIO_USERNAME "/feeds/total";
Adafruit_MQTT_Publish mq_mag_total = Adafruit_MQTT_Publish(&mqtt, MAG_TOTAL_FEED);

/*************************** Sketch Code ************************************/

unsigned long tmr_readsensor;
unsigned long tmr_upload;
unsigned long tmr_printsensor;

void setup() {

  // Init sensor
  //dht.begin();

  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.begin(9600);
  Serial.println(F("Adafruit IO Example"));

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  delay(10);
  Serial.print(F("Connecting to "));
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();

  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());

  
  
  // connect to adafruit io
  connect();
  
  Wire.begin(); //for I2C
  Wire.beginTransmission(addr); //start talking
  Wire.write(0x02); // Set the Register
  Wire.write(0x00); // Tell the HMC5883 to Continuously Measure
  Wire.endTransmission();

  tmr_readsensor = millis();
  tmr_upload = millis();

  digitalWrite(LED_BUILTIN, 1);
  delay(1000);
  digitalWrite(LED_BUILTIN, 0);
  delay(1000);
  digitalWrite(LED_BUILTIN, 1);
  delay(1000);
  digitalWrite(LED_BUILTIN, 0);
  delay(1000);  
  digitalWrite(LED_BUILTIN, 1);
  delay(1000);
  digitalWrite(LED_BUILTIN, 0);
  delay(1000);  
}

int mag_rate_data = 0;
int mag_field_data = 0;     //only look at y axis
int mag_total_data = 0;
const int high_val = 50000;
const int low_val = 18000;


bool high_low = 0;
bool last_high_low;
int x,y,z; //triple axis data


void loop() {
  
  // Grab the current state of the sensor
  //int humidity_data = (int)dht.readHumidity();
  //int temperature_data = (int)dht.readTemperature();


  //***************************************
  // Every 100ms, read sensor data and determine rate and totals
  //***************************************
  if ( (millis() - tmr_readsensor) > 100)   //every 100 ms, read sensor.  Can't go faster than 15Hz  
  {
      tmr_readsensor = millis();
      
      //Tell the HMC what regist to begin writing data into
      Wire.beginTransmission(addr);
      Wire.write(0x03); //start with register 3.
      Wire.endTransmission();
      
     
     //Read the data.. 2 bytes for each axis.. 6 total bytes
      Wire.requestFrom(addr, 6);
      if(6<=Wire.available()){
        x = Wire.read()<<8; //MSB  x 
        x |= Wire.read(); //LSB  x
        z = Wire.read()<<8; //MSB  z
        z |= Wire.read(); //LSB z
        y = Wire.read()<<8; //MSB y
        y |= Wire.read(); //LSB y
      }
      
      
      // Show Values
      //Serial.print("X Value: ");
      //Serial.println(x);
      //Serial.print("Y Value: ");
      //Serial.println(y);
      //Serial.print("Z Value: ");
      //Serial.println(z);
      //Serial.println();
      
      mag_field_data = y;

      if (mag_field_data > high_val)
      {
        high_low = 1;
      }
      else if (mag_field_data < low_val)
      {
        high_low = 0;
      }
      
      if (last_high_low != high_low)
      {
        last_high_low = high_low;
        mag_rate_data++;
        Serial.print("add one:");
        Serial.println(mag_rate_data);
      }
      //digitalWrite(LED_BUILTIN, HIGH);
      digitalWrite(LED_BUILTIN, high_low);
 
  } //end tmr_readsensor


  //***************************************
  // Print sensor data to serial uart
  //***************************************
  if ( (millis() - tmr_printsensor) > 1000)   //every 100 ms, read sensor.  Can't go faster than 15Hz  
  {
      tmr_printsensor = millis();
      
      // Show Values
      Serial.print("X Value: ");
      Serial.println(x);
      Serial.print("Y Value: ");
      Serial.println(y);
      Serial.print("Z Value: ");
      Serial.println(z);
      Serial.println();
 
  } //end tmr_printsensor
  
  
  
  
  //***************************************
  // Upload sensor data to Adafruit
  //***************************************
  if ( (millis() - tmr_upload) > 10000)   //every 10 seconds upload to adafruit
  {
      tmr_upload = millis();

      // ping adafruit io a few times to make sure we remain connected
      if(! mqtt.ping(3)) {
        // reconnect to adafruit io
        if(! mqtt.connected())
          connect();
      }


     
      // Publish data
      if (! mq_mag_field.publish(mag_field_data))
        Serial.println(F("Failed to publish mag_field"));
      else
        {
          Serial.print("published mag field:   ");
          Serial.println(mag_field_data);
          
        }

      if (! mq_mag_rate.publish(mag_rate_data))
        Serial.println(F("Failed to publish humidity"));
      else
        {
          Serial.print("published mag rate:   ");
          Serial.println(mag_rate_data);
        }
      if (! mq_mag_total.publish(mag_total_data))
        Serial.println(F("Failed to publish humidity"));
      else
        {
          Serial.print("published mag total:   ");
          Serial.println(mag_total_data);
        }

      mag_total_data = mag_total_data + mag_rate_data;  //accumulate total
      mag_rate_data = 0;  //reset rate
  }//end tmr for uploading to Adafruit
  
}//end loop


// connect to adafruit io via MQTT
void connect() {

  Serial.print(F("Connecting to Adafruit IO... "));

  int8_t ret;

  while ((ret = mqtt.connect()) != 0) {

    switch (ret) {
      case 1: Serial.println(F("Wrong protocol")); break;
      case 2: Serial.println(F("ID rejected")); break;
      case 3: Serial.println(F("Server unavail")); break;
      case 4: Serial.println(F("Bad user/pass")); break;
      case 5: Serial.println(F("Not authed")); break;
      case 6: Serial.println(F("Failed to subscribe")); break;
      default: Serial.println(F("Connection failed")); break;
    }

    if(ret >= 0)
      mqtt.disconnect();

    Serial.println(F("Retrying connection..."));
    delay(5000);

  }

  Serial.println(F("Adafruit IO Connected!"));

} //end loop
