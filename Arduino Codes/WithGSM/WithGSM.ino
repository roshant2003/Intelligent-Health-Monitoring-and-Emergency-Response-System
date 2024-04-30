//https://api.thingspeak.com/update?api_key=OH1GAUIOMY134WCY&field1=0&field2=1&field3=4&field4=2

// Defining module model
#define TINY_GSM_MODEM_SIM800

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

// Set serial for AT commands (to the module)
#define SerialAT Serial1

// Your GPRS credentials, if any
const char apn[] = "";  //airtelgprs.com
const char gprsUser[] = "";
const char gprsPass[] = "";

// Server details
String api_key = "VTOF5R3V2X6HTIVX"; // Write api key
const char server[] = "api.thingspeak.com";
const int port = 80;

// Imports
#include <TinyGsmClient.h>

// Creting TinyGsm instance
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);


//---------------------------------------

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
uint16_t irBuffer[50]; //infrared LED sensor data
uint16_t redBuffer[50];  //red LED sensor data
uint16_t ecgBuffer[50];
#else
uint32_t irBuffer[50]; //infrared LED sensor data
uint32_t redBuffer[50];  //red LED sensor data
uint32_t ecgBuffer[50];
#endif

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read
int32_t hr_cnt = 0;
int32_t hr_sum = 0;
int32_t sp_cnt = 0;
int32_t ecg_cnt=0;
int32_t sp_sum = 0;
int32_t temp_sum=0;
int32_t ecg_sum =0;

//-----------------------------

#include <Servo.h>

Servo servoMotor;

void setup()
{
  Serial.begin(115200); // initialize serial communication at 115200 bits per second:

  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

//  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
//  while (Serial.available() == 0) ; //wait until user presses a key
//  Serial.read();

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  particleSensor.enableDIETEMPRDY();

//  -----------------------------------


  // Set console baud rate
  SerialMon.begin(115200);

  // Set GSM module baud rate
  SerialAT.begin(9600);

  // Modem initialization
  modem.restart();

  // Display modem information
  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);
  SerialMon.println();

  // Initialize connection to internet
  establishGPRS();

//  --------

  servoMotor.attach(9); 

}

// Establish GPRS Connection
bool establishGPRS() {
  // Connecting to GPRS
  SerialMon.print(F("Connecting to "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println("Fail");
    return false;
  }
  return true;
}

// Connect to GPRS pdate field value in thingspeak using GET
bool sendData(int value1, int value2, int value3, int value4) {  
  // Connecting to server
  SerialMon.print("Connecting to ");
  SerialMon.println(server);
  if (!client.connect(server, port)) {
    SerialMon.println("Failed at sending");
    return false;
  }
  SerialMon.println();

  // Make a HTTP GET request:
  SerialMon.println("Performing HTTP GET request...");
  String resource = "/update?api_key=" + api_key;
  resource += "&field" + String(1) + "=" + String(value1);
  resource += "&field" + String(2) + "=" + String(value2);
  resource += "&field" + String(3) + "=" + String(value3);
  resource += "&field" + String(4) + "=" + String(value4);
  client.print(String("GET ") + resource + " HTTP/1.1\r\n");
  client.print(String("Host: ") + server + "\r\n");
  client.print("Connection: close\r\n\r\n");
  client.println();

  uint32_t timeout = millis();
  while (client.connected() && millis() - timeout < 10000L) {
    // Print available data
    while (client.available()) {
      char c = client.read();
      SerialMon.print(c);
      timeout = millis();
    }
  }
  SerialMon.println();

  // Disconnect from server
  client.stop();
  SerialMon.println("Server disconnected");
  return true;
}


int ECGSense(){
  if((digitalRead(10) == 1)||(digitalRead(11) == 1)){
    return -1;
  }
  else{
    // send the value of analog input 0:
    int ecg_reading = analogRead(A0);
//    Serial.print("ECG : ");
//    Serial.println(ecg_reading);
    //Wait for a bit to keep serial data from saturating
    delay(1);
    
    return ecg_reading;
  }
  
  delay(100);
}

void loop()
{
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
//    ecgBuffer[i] = ECGSense();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

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
//      ecgBuffer[i-25] = ecgBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
//      ecgBuffer[i] = ECGSense();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      float temperature = particleSensor.readTemperature();

      float temperatureF = particleSensor.readTemperatureF(); //Because I am a bad global citizen

      if (validSPO2) {
        sp_cnt++;
        sp_sum+=spo2;
        temp_sum+=temperatureF;
      }
      if (validHeartRate) {
        hr_cnt++;
        hr_sum+=heartRate;
      }

      int ecg_reading=ECGSense();
      
      if(ecg_reading!=-1){
        ecg_cnt++;
        ecg_sum+=ecg_reading;
      }
      
      
      if (hr_cnt > 100 && sp_cnt > 100) {

        float averageSPO2 = static_cast<float>(sp_sum) / sp_cnt;
        float averageHR = static_cast<float>(hr_sum) / hr_cnt;
        float averageTemp = static_cast<float>(temp_sum) / sp_cnt;
        float averageECG = static_cast<float>(ecg_sum) / ecg_cnt;

//        Serial.println();
//        Serial.println("------ Averages ------");
        Serial.print("Average SpO2:");
        Serial.print(averageSPO2, 2);
        Serial.print(";");
        Serial.print("Average Heart Rate:");
        Serial.print(averageHR, 2);
        Serial.print(";");
        Serial.print("Average Temperature :");
        Serial.print(averageTemp, 2);
        Serial.print(";");
        Serial.print("Average ECG:");
        Serial.print(averageECG, 2);
        Serial.print(";");
        Serial.println("-----END-----");


        bool res=modem.isGprsConnected();
        if (res){
          sendData(averageECG, averageHR, averageSPO2, averageTemp);
        }

////         Servo logic
//    if (Serial.available() > 0) {
//        int angle = Serial.parseInt(); // Read the angle from serial
//        Serial.print("Servo : ");
//        servoMotor.write(angle); // Move the servo to the specified angle
//    }
      }
    }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
}
