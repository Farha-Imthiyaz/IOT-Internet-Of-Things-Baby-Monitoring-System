/*
Title: IoT Based Baby Monitoring System 
Developed by: Farha Imthiyaz 
Sensors Used: DHT21, Keys photo resister (Light Intensity)and sound Sensor
Actuators Used: Mini fan, two LED bulbs and Speaker
IoT Platforms: Blynk
*/
#include "DHT.h"
#define BLYNK_PRINT Serial
#define EspSerial Serial3
#define ESP8266_BAUD 9600

ESP8266 wifi(&EspSerial);

#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>

//DHT Sensor
#define pin_DHT 2  
#define DHTTYPE DHT21
float v_TemperatureValue;
float v_HumidityValue;
DHT dht(pin_DHT, DHTTYPE);

//Light Intensity Sensor
#define pin_lightintensity A0;
int v_lightIntensityValue;

//Sound Sensor
#define pin_sound 15;
int v_soundValue;

//Fan
#define pin_fan 13;

//Two LED bulbs
#define pin_ledred 12;
#define pin_ledgreen 14;

//Speaker
#define pin_speaker 2;

//Blynk
char auth[] = "ZGT1tZr6p1GpNPZd-7-OojinYvAK3_9I";
char ssid[] = "Dialog 4G 403";
char pass[] = "MyRabbRahman";


void dhtSensor()
{
    float h = dht.readHumidity(); //Humidity
    float t = dht.readTemperature(); //Temperature Celsius
    float f = dht.readTemperature(true); //Temperature Fahrenheit  
  
    //Sensor Status Checking
    if (isnan(h) || isnan(t) || isnan(f))
    {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }
    
    v_TemperatureValue=t;
    v_HumidityValue=h;
  
    String strTempCelsius = "Temperature: " + String(t) + "*C";
    String strHumidity= "Humidity :" + String(h);
  
    Serial.println(strTempCelsius);
    Serial.println(strHumidity);      
}

void lightIntensity()
{
    v_lightIntensityValue = analogRead(pin_lightintensity); 
    Serial.println(v_lightIntensityValue, DEC); // light intensity
                  // high values for bright environment
                  // low values for dark environment
    delay(1000); 
}

void soundSensor()
{
    v_soundValue = digitalRead(pin_sound);
    // print out the value you read:
    Serial.print("Sound detected: ");
    Serial.println(v_soundValue);
    delay(1000);        // delay in between reads for stability
}

BlynkTimer timer;
void myTimerEvent()
{
    dhtSensor();
    lightIntensity();
    soundSensor();
      
    //V1 Pin Temperature Sensor Value
    Blynk.virtualWrite(V1, v_TemperatureValue);
  
    //V2 Light Intensity Sensor Value
    Blynk.virtualWrite(V2, v_lightIntensityValue);
  
    //V3 Sound Sensor Value
    Blynk.virtualWrite(V3, v_soundValue);  
      
}

//from V4 button we are controlling fan
BLYNK_WRITE(V4)
{
    int pinValueV4 = param.asInt(); // assigning incoming value from pin V4 to a variable
    // You can also use:
    // String i = param.asStr();
    // double d = param.asDouble();
    Serial.print("V4 Button value is: ");
    Serial.println(pinValueV4);
  
    if(pinValueV4==1)
    {
      Serial.println("Fan On");
      digitalWrite(pin_fan, HIGH);   
    }
    if(pinValueV4==0)
    {
      Serial.println("Fan Off");
      digitalWrite(pin_fan, LOW);    
    }        
}

//from V5 button we are controlling two LEDs
BLYNK_WRITE(V5)
{
    int pinValueV5 = param.asInt(); // assigning incoming value from pin V5 to a variable
    // You can also use:
    // String i = param.asStr();
    // double d = param.asDouble();
    Serial.print("V5 Button value is: ");
    Serial.println(pinValueV5);
  
    if(pinValueV5==1)
    {
      Serial.println("Both LED On");
      digitalWrite(pin_ledred, HIGH);
      digitalWrite(pin_ledgreen, HIGH);   
    }
    if(pinValueV5==0)
    {
      Serial.println("Both LED Off");
      digitalWrite(pin_ledred, LOW);
      digitalWrite(pin_ledgreen, LOW);    
    }        
}

//from V6 button we are controlling Speaker
BLYNK_WRITE(V6)
{
    int pinValueV6 = param.asInt(); // assigning incoming value from pin V5 to a variable
    // You can also use:
    // String i = param.asStr();
    // double d = param.asDouble();
    Serial.print("V6 Button value is: ");
    Serial.println(pinValueV6);
  
    if(pinValueV6==1)
    {
      Serial.println("Speaker On");
      digitalWrite(pin_speaker, HIGH);     
    }
    if(pinValueV6==0)
    {
      Serial.println("Speaker Off");
      digitalWrite(pin_speaker, LOW);   
    }        
}

void setup()
{
    Serial.begin(9600); 
    pinMode(pin_fan, OUTPUT);
    pinMode(pin_ledred,OUTPUT);
    pinMode(pin_ledgreen,OUTPUT);
    pinMode(pin_speaker,OUTPUT);
    
    dht.begin();
    EspSerial.begin(ESP8266_BAUD);
    delay(10);
    Blynk.begin(auth, wifi, ssid, pass);
    timer.setInterval(1000L, myTimerEvent);  
}

void loop()
{
    Blynk.run();
    timer.run(); // Initiates BlynkTimer       
}
