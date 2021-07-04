
/*
1) This shows a live human Heartbeat Pulse.
2) Live visualization in Arduino's Cool "Serial Plotter".
3) Blink an LED on each Heartbeat.
4) This is the direct Pulse Sensor's Signal.
5) A great first-step in troubleshooting your circuit and connections."

*/
#include <ESP8266WiFi.h>
#include "wifi_config.h"
#include <ThingSpeak.h>

const char* ssid = WIFI_SSID;
const char* pwd = WIFI_PWD;

unsigned long myChannelNumber = CH_ID;
const char * myWriteAPIKey = API_KEY;

WiFiClient client;


//  Variables
int PulseSensorPin = 0;        // Pulse Sensor PURPLE WIRE connected to ANALOG PIN 0
int LED16 = 16;   //  The on-board Arduion LED

int Threshold = 550;            // Determine which Signal to "count as a beat", and which to ingore.
int bpm_avg = 75;

float pulse_max = Threshold; 
float pulse_min = Threshold;
unsigned long last_update_threshold = 0;
unsigned long last_update_thingspeak = millis();


// The SetUp Function:
void setup() {
  pinMode(LED16,OUTPUT);         // pin that will blink to your heartbeat!
  Serial.begin(9600);            // Set's up Serial Communication at certain speed.

  Serial.println("Connecting to ");
  Serial.println(ssid); 

  ThingSpeak.begin(client);
}

void connect(){
    if(WiFi.status() != WL_CONNECTED){
    while (WiFi.status() != WL_CONNECTED) 
    {
      WiFi.begin(ssid, pwd); 
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected"); 
  }
}

// The Main Loop Function
void loop() {
  // Read the PulseSensor's value.

  int count = 0;
  int bpm=0;
  unsigned long bpm_start_time = millis();

  if(WiFi.status() != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    while(WiFi.status() != WL_CONNECTED){
      WiFi.begin(ssid, pwd);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      Serial.print(".");
      delay (10000);
    } 
  }

  int PulseRead = analogRead(PulseSensorPin);

  setThreshold(PulseRead);
  
  bool sawBeatBegin = false;
  while(true){
    PulseRead = analogRead(PulseSensorPin);
    setThreshold(PulseRead);

    if(PulseRead>Threshold  && pulse_max - pulse_min>11){
      digitalWrite(LED16, LOW);
      if(sawBeatBegin==false){
        count++;
        Serial.print("Beats: ");
        Serial.println(count);
        sawBeatBegin = true;
      }
    } else if(PulseRead<=Threshold){
      digitalWrite(LED16, HIGH);
      sawBeatBegin = false;
    }
    
    
    if(millis()-bpm_start_time > 30000){
      Serial.print("The average pulse rate in the past 30 sec is: ");
      bpm = count*2;
      Serial.println(bpm);
      bpm_avg = (bpm_avg+bpm)/2;
      bpm_start_time = millis();
      count = 0;
    }

    if(millis()-last_update_thingspeak > 40000){
      Serial.println("Uploading data to thingspeak");
      ThingSpeak.setField(1, bpm);
      ThingSpeak.setField(2, bpm_avg);
      String bpm_status;
      if(bpm<30){
        bpm_status = String("offline");
        Serial.println("Going to sleep for 300 seconds");
        ESP.deepSleep(300e6);
      }
      else{
        bpm_status = String("online");
      }
      ThingSpeak.setStatus(bpm_status);

      // write to the ThingSpeak channel
      int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
      if(x == 200){
        Serial.println("Channel update successful.");
      }
      else{
        Serial.println("Problem updating channel. HTTP error code " + String(x));
      }
      last_update_thingspeak = millis();

    }  
  }

  


}

void setThreshold(int Signal){

  if(Signal > pulse_max){
    pulse_max = Signal;   
  }
  else if(Signal < pulse_min){
    pulse_min = Signal;
  }

  if(millis()-last_update_threshold>1000){
    Threshold = (4*pulse_min/9)+(5*pulse_max/9);
    last_update_threshold = millis();
    pulse_min = Threshold;
    pulse_max = Threshold;
  }
}
