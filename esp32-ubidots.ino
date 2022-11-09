#include "UbidotsEsp32Mqtt.h"
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <LiquidCrystal_I2C.h>

#define REPORTING_PERIOD_MS     1000
uint32_t tsLastReport = 0;

MAX30105 particleSensor;
#define MAX_BRIGHTNESS 255 
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read 

#define ADC_VREF_mV    3300.0 // in millivolt
#define ADC_RESOLUTION 4096.0
#define PIN_LM35       35 // ESP32 pin GIOP36 (ADC0) connected to LM35
 
int adcVal; 
float milliVolt;
float tempC; 

LiquidCrystal_I2C lcd(0x3F,16,2);

/****************************************

 * Define Constants

 ****************************************/

const char *UBIDOTS_TOKEN = "BBFF-UJYY72S8bCVVoFDjpx6vaGzOggDq9X";  // Put here your Ubidots TOKEN

const char *WIFI_SSID = "GNXS-2EC0D0";      // Put here your Wi-Fi SSID

const char *WIFI_PASS = "9629090034";      // Put here your Wi-Fi password

const char *DEVICE_LABEL = "PatientHealthMonitoring";   // Put here your Device label to which data  will be published

const char *VARIABLE_LABEL = "BodyTemperature";

const char *VARIABLE_LABEL2 = "Heart Rate";// Put here your Variable label to which data  will be published

const char *VARIABLE_LABEL3 = "Blood Oxygen";

const int PUBLISH_FREQUENCY = 5000; // Update rate in milliseconds



unsigned long timer;


Ubidots ubidots(UBIDOTS_TOKEN);



/****************************************

 * Auxiliar Functions

 ****************************************/



void callback(char *topic, byte *payload, unsigned int length)

{

  Serial.print("Message arrived [");

  Serial.print(topic);

  Serial.print("] ");

  for (int i = 0; i < length; i++)

  {

    Serial.print((char)payload[i]);

  }

  Serial.println();

}



/****************************************

 * Main Functions

 ****************************************/



void setup()

{

  // put your setup code here, to run once:

  Serial.begin(115200);
  pinMode(19, OUTPUT);
  delay(100);

  lcd.init();         // initialize the lcd

  lcd.backlight();    // open the backlight  


  // ubidots.setDebug(true);  // uncomment this to make debug messages available

  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);

  ubidots.setCallback(callback);

  ubidots.setup();

  ubidots.reconnect();

  timer = millis();

  Serial.print("Initializing pulse oximeter..");
 
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);
 
  // Initialize max30102 sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30102 was not found. Please check wiring/power."));
    //while (1);
  }
 
  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  while (Serial.available() == 0) ; //wait until user presses a key
  Serial.read();
 
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
 
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
 
}



void loop()

{

  // put your main code here, to run repeatedly:

  if (!ubidots.connected())

  {

    ubidots.reconnect();

  }

  if (abs(millis() - timer) > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds

  {

    // read the ADC value from the temperature sensor
  adcVal = analogRead(PIN_LM35);
  // convert the ADC value to voltage in millivolt
  milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
  // convert the voltage to the temperature in °C
  tempC = milliVolt / 14;



  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps
 
  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data
 
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
 
    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }
 
  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
 
  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (1)
  {
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }
 
    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data
 
      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read
 
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample
 
      //send samples and calculation result to terminal program through UART
      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.print(irBuffer[i], DEC);
 
      Serial.print(F(", HR="));
      Serial.print(heartRate, DEC);
 
      Serial.print(F(", HRvalid="));
      Serial.print(validHeartRate, DEC);
 
      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);
 
      Serial.print(F(", SPO2Valid="));
      Serial.println(validSPO2, DEC);

      if (millis() - tsLastReport > REPORTING_PERIOD_MS) 
  {

 
     
    
    Serial.print("BPM: ");
    Serial.println(heartRate);
    
    Serial.print("SpO2: ");
    Serial.print(spo2);
    Serial.println("%");

    // print the temperature in the Serial Monitor:
    Serial.print("Temperature: ");
    Serial.print(tempC);   // print the temperature in °C
    Serial.println("°C");

     if (isnan(tempC) || isnan(heartRate) || isnan(spo2)) {
    lcd.setCursor(0, 0);
    lcd.print("Failed");
  } else {
    lcd.setCursor(0, 1);  // display position
    lcd.print("Temp: ");
    lcd.print(tempC);     // display the temperature
    lcd.print("C");
    delay(1000);
    lcd.clear();

    lcd.setCursor(1, 1);  // display position
    lcd.print("HR: ");
    lcd.print(heartRate);      // display the humidity
    lcd.print("BPM");
    delay(1000);
    lcd.clear();
    
    lcd.setCursor(2, 1);  // display position
    lcd.print("SPO2: ");
    lcd.print(spo2);      // display the humidity
    lcd.print("%");
    delay(1000);
    lcd.clear();

  

  

     }
  }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  
   
   

    delay(2000);

    ubidots.add(VARIABLE_LABEL, tempC);

    ubidots.add(VARIABLE_LABEL2, heartRate);

    ubidots.add(VARIABLE_LABEL3, spo2);

    ubidots.publish(DEVICE_LABEL);

    timer = millis();

  }
  }
  }
  

  ubidots.loop();

}