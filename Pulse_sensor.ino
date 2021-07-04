
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
int LED16 = 16;               //  The on-board Arduion LED

int ThresholdBPM = 550;       // Determine which Signal to "count_bpm as a beat", and which to ingore.
int bpm_avg = 75;

float pulse_max = ThresholdBPM; 
float pulse_min = ThresholdBPM;
unsigned long last_update_threshold = 0;
unsigned long last_update_thingspeak = millis();


// The SetUp Function:
void setup() {
  pinMode(LED16,OUTPUT);         // pin that will blink to your heartbeat!
  Serial.begin(9600);            // Set's up Serial Communication at certain speed. (9600baud)

  Serial.println("Connecting to ");
  Serial.println(ssid); 

  ThingSpeak.begin(client);
}


// The Main Loop Function
void loop() {

  int count_bpm = 0;  //track
  int bpm=0;
  unsigned long bpm_start_time = millis();

  connect_wifi();

  int PulseRead = analogRead(PulseSensorPin);

  setThresholdBPM(PulseRead);
  
  bool sawBeatBegin = false;
  while(true){
    PulseRead = analogRead(PulseSensorPin);
    setThresholdBPM(PulseRead);

    if(PulseRead>ThresholdBPM  && pulse_max - pulse_min>11){
      digitalWrite(LED16, LOW);
      if(sawBeatBegin==false){
        count_bpm++;
        Serial.print("Beats: ");
        Serial.println(count_bpm);
        sawBeatBegin = true;
      }
    } else if(PulseRead<=ThresholdBPM){
      digitalWrite(LED16, HIGH);
      delay(100);
      sawBeatBegin = false;
    }
    
    
    if(millis()-bpm_start_time > 30000){
      Serial.print("The average pulse rate in the past 30 sec is: ");
      bpm = count_bpm*2;
      Serial.println(bpm);
      bpm_avg = (bpm_avg+bpm)/2;
      bpm_start_time = millis();
      count_bpm = 0;
      break;
    }

  }
  sendToThingSpeak(bpm);
}

/*
Code for connecting to WiFi Network.
*/
void connect_wifi(){
  if(WiFi.status() != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    while(WiFi.status() != WL_CONNECTED){
      WiFi.begin(ssid, pwd);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      Serial.print(".");
      delay (10000);
    } 
  }
}

/*
Send Data to thingspeak.
*/
void sendToThingSpeak(int bpm){
  if(millis()-last_update_thingspeak > 30000){
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

/*
Set Threshold for calculating BPM. 
*/
void setThresholdBPM(int Signal){

  if(Signal > pulse_max){
    pulse_max = Signal;   
  }
  else if(Signal < pulse_min){
    pulse_min = Signal;
  }

  if(millis()-last_update_threshold>1000){
    ThresholdBPM = (4*pulse_min/9)+(5*pulse_max/9);
    last_update_threshold = millis();
    pulse_min = ThresholdBPM;
    pulse_max = ThresholdBPM;
  }
}
