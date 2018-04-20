/* Code to validate the Microphone usage for FFT 
 *  This code is dedicated to ESP32
 *  
 */


#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"`
#include "OLEDDisplay.h"
#include <rom/rtc.h>
#include <PlainFFT.h>

#define ACT_LED 13
const int analogInPin = A0;  // Analog input pin

QueueHandle_t queue;

SSD1306  display(0x3c,23,22);

TaskHandle_t hCollectTask;

// define playload sent over queue

#define DATASIZE 128
#define QUEUSIZE 10

struct Spectrum {
  unsigned long timestamp;
  int data[DATASIZE];
};

PlainFFT FFT = PlainFFT(); // Create FFT object

double signalFrequency = 50;
double samplingFrequency = 1000;
uint8_t signalIntensity = 100;
double vReal[DATASIZE]; 
double vImag[DATASIZE];


void collectTask (void *args) { // TASK THAT COLLECT SIGNAL AND TRANSMIT IT
  struct Spectrum mesure;
  unsigned long sampleCount=0;
  
  while (true) {
    int sensorValue = 0; 
    mesure.timestamp=millis();
    // HERE IS A SIMULATION of data acquisition

    // REAL ADC implementation 
    while (sampleCount<DATASIZE){ // Aquire DATASIZE samples
      mesure.data[sampleCount] = analogRead(analogInPin);
      delay(1);
      sampleCount++;
    }
    sampleCount=0;
  //  Serial.printf("COLLECTER [core# %d] Collect task iteration... ",xPortGetCoreID());    
    xQueueSend(queue,(void*)&mesure,  (TickType_t)0 );
  //  Serial.printf("data sent (timestamp is %d ms)\n",mesure.timestamp);    

//    delay (500);    
  }
}


void setup() { // INITIALIZE THE SYSTEM
  Serial.begin(115200);
  delay(1000);
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0,0,"BOOTING Spectrum analysis");
  display.display();    

  pinMode(ACT_LED,OUTPUT);
  digitalWrite (ACT_LED,LOW);
  
  Serial.printf("SETUP CPU 0 reset reason %d\n",rtc_get_reset_reason(0));
  Serial.printf("SETUP CPU 1 reset reason %d\n",rtc_get_reset_reason(1));
  Serial.println("SETUP in progress...");
  Serial.println("SETUP collect task");
  queue = xQueueCreate(QUEUSIZE, sizeof(Spectrum));

  xTaskCreatePinnedToCore( // start RT task
    collectTask,
    "collect",
    4096,
    NULL,
    1,
    &hCollectTask,
    0);
  Serial.println("SETUP DONE");
}

/*
 * Receive signal process and display
 */
void loop() { 
  struct Spectrum mesure;
  String json;
  digitalWrite (ACT_LED,LOW);
  if (queue != NULL){    
    if (xQueueReceive(queue, &mesure, (TickType_t)10) == pdTRUE){
      digitalWrite (ACT_LED,HIGH);

 //     Serial.printf("RECEIVER  [core# %d] Someting to read in the queue: ",xPortGetCoreID());
 //     Serial.printf("received timestamp %d\n", mesure.timestamp);
      // FFT PROCESS
      
      int mMax=0;
      for (int i=0; i<DATASIZE; i++) {
        if (mesure.data[i]>mMax) mMax=mesure.data[i];
        vReal[i]=mesure.data[i]?mesure.data[i]:0.00001;
      }
      FFT.windowing(vReal, DATASIZE);  // Weigh data
      FFT.compute(vReal, vImag, DATASIZE, FFT_FORWARD); // Compute FFT
      FFT.complexToMagnitude(vReal, vImag, DATASIZE); // Compute magnitudes
      // get max value
      double vmax=0;
      for (int i=2; i<(DATASIZE / 2); i++) {
        if (vReal[i] > vmax) vmax=vReal[i];
      }
      
      
      if (mMax>0) { /// somthing to display
      // Display data
        double peak=FFT.majorPeak(vReal, DATASIZE, samplingFrequency);
        
        json="{\"timestamp\":\""+String(mesure.timestamp)+"\",\"majorPeak\":\""+peak+"\",\"data\":[";
        display.clear();
        for (int i=0; i<(DATASIZE / 2); i++) {
          display.drawVerticalLine(i*2,64-map((int)vReal[i],0,(int)vmax,0,50), 64);
          if(i>0) json+=", ";
          json+=("\""+String(vReal[i])+"\"");
        }
        json+="]}";
//        Serial.print("\n"); Serial.print(json); Serial.print ("\n");
        display.setFont(ArialMT_Plain_10); 
        display.drawString(0,0,"p="+String(peak,1)+"Hz max="+String(vmax,1));
        display.display();  
      } else {
        display.clear();
        display.setFont(ArialMT_Plain_16); 
        display.drawString(0,0,"NO SIGNAL");
        display.display();  
      }
    }
  }
}
