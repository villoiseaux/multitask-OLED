/* Code to validate the Microphone usage for FFT 
 *  This code is dedicated to ESP32
 *  
 */


#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"`
#include "OLEDDisplay.h"
#include <rom/rtc.h>
#include <PlainFFT.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <PubSubClient.h>

//#define WIFI_ENABELED

const char* server = "mare.pion-hebert.fr";

void mqttCallback(char* topic, byte* payload, unsigned int length);

WiFiClient wifiClient;

PubSubClient client(wifiClient);

const char* ssid     = "SFO";
const char* password = "Smartlab2018";

const char* mqttServer = "mare.pinon-hebert.fr";
const int mqttPort = 1883;
const char* mqttUser = "jpinon";
const char* mqttPassword = "f588rmp";

#define ACT_LED 13
const int analogInPin = A3;  // Analog input pin (BEWARE OF A0! It dosen't work while wifi's running

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

double squelch = 4; /// minimum peak value to display 
double samplingFrequency = 200; // in Hz (1 acq = (1/400)*DATASIZE )
unsigned long delayBetweenAcq = 0; // in milli seconds
unsigned long sampleDelay=(1000*1000/samplingFrequency);
unsigned long sampleCount = 235; // 5 min=467
uint8_t signalIntensity = 100;
double vReal[DATASIZE]; 
double vImag[DATASIZE];


void mqttCallback(char* topic, byte* payload, unsigned int length) {
  #ifdef WIFI_ENABELED
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  #endif
}

void collectTask (void *args) { // TASK THAT COLLECT SIGNAL AND TRANSMIT IT
  struct Spectrum mesure;
  unsigned long sampleCount=0;
  Serial.println("Starting COLLECT task");
  vTaskDelay(1000 * portTICK_PERIOD_MS);
  Serial.println("COLLECT task started");
  adcAttachPin(analogInPin);
  adcStart(analogInPin); 
  while (true) {
    digitalWrite (ACT_LED,HIGH);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    int sensorValue = 0; 
    mesure.timestamp=millis();
    // REAL ADC implementation 
    while (sampleCount<DATASIZE){ // Aquire DATASIZE samples
      //mesure.data[sampleCount] = analogRead(analogInPin);            
      while (adcBusy(analogInPin)) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }
      mesure.data[sampleCount] = adcEnd(analogInPin);
      adcStart(analogInPin);
      ets_delay_us(sampleDelay);
      sampleCount++;
    }
    sampleCount=0;    
    digitalWrite (ACT_LED,LOW);
    xQueueSend(queue,(void*)&mesure,  (TickType_t)0 );
    vTaskDelay(delayBetweenAcq / portTICK_PERIOD_MS);
  }
}


bool mqttConnect() {
  #ifdef WIFI_ENABELED
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
 
    if (client.connect("ESP32Client", mqttUser, mqttPassword )) {
 
      Serial.println("connected");
      return (true);
    } else {
 
      Serial.print("failed with state ");
      Serial.print(client.state());
      return (false);
      delay(2000);
 
    }
  }  
  #endif
  return (true);
}

void setup() { // INITIALIZE THE SYSTEM
  Serial.begin(115200);
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0,0,"Firmware update...");
  display.display();    
  delay (500);
  display.drawString(0,0,"Booting");
  display.display();    
  pinMode(ACT_LED,OUTPUT);
  digitalWrite (ACT_LED,LOW);
  
  Serial.printf("SETUP CPU 0 reset reason %d\n",rtc_get_reset_reason(0));
  Serial.printf("SETUP CPU 1 reset reason %d\n",rtc_get_reset_reason(1));
  Serial.println("SETUP in progress");
  Serial.println("SETUP collect task");
  queue = xQueueCreate(QUEUSIZE, sizeof(Spectrum));

  xTaskCreatePinnedToCore( // start RT task
    collectTask,
    "collect",
    4096,
    NULL,
    0,
    &hCollectTask,
    0); // here is the core


  #ifdef WIFI_ENABELED
    Serial.println("Start wifi");
    display.drawString(0,12,"Start WIFI");
    display.display();    
    WiFi.begin(ssid, password);
    display.display();    
    while (WiFi.status() != WL_CONNECTED) {
      display.clear();
      display.drawString(24,12,"WIFI");
      display.display();    
      delay(250);
      display.clear();
      display.display();    
      delay(250);    
      Serial.print(".");    
    }
    randomSeed(micros());
    Serial.println("Wifi OK");
    Serial.println("SETUP DONE");
    display.clear();
    display.drawString(0,24,"WIFI OK");
    display.display();    
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    
    Serial.println("Contact cloud");
    client.setServer(mqttServer, mqttPort);
   
  
    if (mqttConnect()){
      client.publish("jpinon/message", "Module started");
    }
  #else
    Serial.println("WIFI DISABLED IN COMILATION OPTIONS");
  #endif
}

/*
 * Receive signal process and display
 */

double sum[DATASIZE / 2];
unsigned long svgCount=sampleCount;
 
void loop() { 
  struct Spectrum mesure;
  String json;
  #ifdef WIFI_ENABELED
  if (WiFi.status() == WL_CONNECTED){    
  #endif
    vTaskDelay(10 / portTICK_PERIOD_MS);

    if (queue != NULL){    
      if (xQueueReceive(queue, &mesure, (TickType_t)10) == pdTRUE){
        int mMax=0;
        for (int i=0; i<DATASIZE; i++) {
          if (mesure.data[i]>mMax) mMax=mesure.data[i];
          vReal[i]=mesure.data[i]?mesure.data[i]:0.00001;
        }
        FFT.windowing(vReal, DATASIZE);  // Weigh data
        FFT.compute(vReal, vImag, DATASIZE, FFT_FORWARD); // Compute FFT
        FFT.complexToMagnitude(vReal, vImag, DATASIZE); // Compute magnitudes
        // Filter 154Hz
/*        vReal[48]=vReal[48]/3.8;
        vReal[49]=vReal[49]/137.8/32;
        vReal[50]=vReal[50]/109.6/32;
        vReal[51]=vReal[51]/1.9;*/
        // get max value
        double vmax=0;
        for (int i=2; i<(DATASIZE / 2); i++) {
          if (vReal[i] > vmax) vmax=vReal[i];
        }
              
        if (mMax>0) { /// somthing to process        
          double peak=FFT.majorPeak(vReal, DATASIZE, samplingFrequency);
          if (vmax >squelch) { // DISPLAY GRAPH
            display.clear();
            for (int i=2; i<(DATASIZE / 2); i++) {            
              display.drawVerticalLine(i*2,54-map((int)vReal[i],0,(int)vmax,0,40), map((int)vReal[i],0,(int)vmax,0,40));              
            }
            //Serial.println(json);
            display.setFont(ArialMT_Plain_10); 
            display.drawString(0,0,"p="+String(peak,1)+"Hz max="+String(vmax,1));
            display.drawRect(0,57,127,6);
            display.fillRect(2,59,map(sampleCount-svgCount,0,sampleCount,2,123),2);
            display.display();  

            // SEND ONLY ID sampleCount reached
            if (svgCount>1){
              // add sample
              for (int i=0; i<(DATASIZE / 2); i++) { 
                sum[i]+=(int)vReal[i];
              }
              svgCount--;
            } else {
              // send spectrum
              if (mqttConnect()){
                json="{\"freqAcq\":"+String(samplingFrequency)+",\"timestamp\":"+String(mesure.timestamp)+",\"majorFreq\":"+peak+"\",\"majorVal\":"+vmax+",\"squelch\":"+squelch+",\"data\":[";
                for (int i=0; i<(DATASIZE / 2); i++) {            
                  if(i>0) json+=", ";
                  json+=("\""+String(sum[i]/sampleCount)+"\"");
                }
                json+="]}";
                char buf[1000];
                json.toCharArray(buf,1000);
                #ifdef WIFI_ENABELED
                  client.publish("jpinon/mesure/spectrum", buf);
                #endif
              }
              for (int i=0; i<(DATASIZE / 2); i++) {  
                sum[i]=0;
              }
              svgCount=sampleCount;
            }
            
          } else {
             
            display.clear();
            display.setFont(ArialMT_Plain_16); 
            display.drawString(20,32-8,"low signal");
            display.drawString(16,48-8,"(squelch "+String(squelch)+")");
            display.display();  
          }
        } else {
          display.clear();
          display.setFont(ArialMT_Plain_16); 
          display.drawString(0,0,"NO SIGNAL");
          display.display();  
        }
      } 
    }
  #ifdef WIFI_ENABELED   
  } else {
    Serial.println(".");
  }
  #endif
}
