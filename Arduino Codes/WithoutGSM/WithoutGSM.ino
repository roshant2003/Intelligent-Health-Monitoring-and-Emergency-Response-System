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

      }
    }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
}
