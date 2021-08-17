
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include "wifi_config.h"
#include <ThingSpeak.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

//Creating objects for managing MPU6050 and BMP280 sensors.
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;

//Sensor events for MPU6050
sensors_event_t a, g, temp;

//WiFi Credentials (defined in wificonfig.h)
const char *ssid = WIFI_SSID;
const char *pwd = WIFI_PWD;

//URL for local django server
String localServerName = "http://192.168.0.100:8080/send";

//ThingSpeak Credentials
unsigned long myChannelNumber = CH_ID; //ThingSpeak Channel ID
const char *myWriteAPIKey = API_KEY;   //ThingSpeak API_Key

//WiFiClient object
WiFiClient client;
//HTTPClient object
HTTPClient http_client;

// Global Variables
int PulseSensorPin = 0; // Pulse Sensor PURPLE WIRE connected to ANALOG PIN 0
int LED2 = 2;           // The NodeMCU on-board LED
int ThresholdBPM = 550; // Determine which Signal portion of the signal to count and which to ingore .
int bpm_avg = 75;       // Average Pulse Rate (Measured in Beats Per Minute or BPM)
int bpm_curr = 0;       // Current Pulse Rate (Measured in Beats Per Minut or BPM)

float pulse_max = ThresholdBPM;                      //Max value of signal seen during each cycle of setThresholdBPM
float pulse_min = ThresholdBPM;                      //Min value of signal seen during each cycle of setThresholdBPM
unsigned long last_update_threshold = 0;             //Time in milliseconds of the last threshold update
unsigned long last_update_thingspeak_bpm = millis(); //Time in milliseconds of last thingspeak update for pulse
unsigned long last_update_thingspeak_acc = millis(); //Time in milliseconds of last thingspeak update for acceleration

float base_height; // Height of ground above sea level
float curr_height; // Current height above sea level

bool fall_detected = true; // Stores weather a fall has been detected or not
String address = "\"0\"";  // Address of this sensor node

/*Function Prototypes*/

void connect_wifi();
void handleAcceleration();
void sendToThingSpeak(int bpm);
void sendFallToThingspeak(unsigned long fall_dur);
void setThresholdBPM(int Signal);
void setMPUProperties(Adafruit_MPU6050 &mpu);
void send_json(String fall_occurred);

// The SetUp Function:
void setup()
{
  pinMode(LED2, OUTPUT); // Light blinks on each pulse.
  Serial.begin(9600);    // Set's up Serial Communication at certain speed. (9600baud)
  digitalWrite(D0, LOW); // Initially the LED of nodeMCU is turned on

  //Initialize serial
  while (!Serial)
  {
    delay(10);
  }

  // Initialize mpu6050
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }

  // Initialize bmp280
  if (!bmp.begin())
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1)
    {
      delay(1000);
    };
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  ThingSpeak.begin(client); //Initialise ThingSpeak
  base_height = bmp.readAltitude(1010.66f);
  Serial.print("Altitude above sea level: ");
  Serial.println(base_height);
  Serial.print("Connecting to server: ");
  setMPUProperties(mpu);
}

// The Main Loop Function
void loop()
{
  int count_bpm = 0;                       //count heartbeats
  int bpm = 0;                             //value of bpm
  unsigned long bpm_start_time = millis(); //Time at which the bpm measurement started
  unsigned long beat_begin_time;           //Time at which the last beat started

  connect_wifi();

  int PulseRead = analogRead(PulseSensorPin);

  // setThresholdBPM(PulseRead);
  ThresholdBPM = 150;

  bool sawBeatBegin = false;
  while (true)
  {
    curr_height = bmp.readAltitude(1010.66f);

    PulseRead = analogRead(PulseSensorPin);
    setThresholdBPM(PulseRead);

    mpu.getEvent(&a, &g, &temp);

    handleAcceleration();
    if (PulseRead > ThresholdBPM && pulse_max - pulse_min > 6 && millis() - beat_begin_time > 190)
    {
      digitalWrite(LED2, LOW);
      if (sawBeatBegin == false)
      {
        count_bpm++;
        Serial.print("Beats: ");
        Serial.println(count_bpm);
        sawBeatBegin = true;
        beat_begin_time = millis();
      }
    }
    else if (PulseRead <= ThresholdBPM)
    {
      digitalWrite(LED2, HIGH);
      delay(100);
      sawBeatBegin = false;
    }

    if (millis() - bpm_start_time > 60000)
    {
      Serial.print("The average pulse rate in the past 30 sec is: ");
      bpm = count_bpm;
      bpm_curr = bpm;
      Serial.println(bpm);
      bpm_avg = (bpm_avg + bpm) / 2;
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
void connect_wifi()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Connecting to ");
    Serial.println(ssid);

    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    while (WiFi.status() != WL_CONNECTED)
    {
      WiFi.begin(ssid, pwd); // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      Serial.print(".");
      delay(10000);
    }
    last_update_threshold = millis();
    last_update_thingspeak_bpm = millis();
  }
}

/*
 * Reads acceleration events, and trigger sending data to local 
 * django webserver and thingspeak in case a fall is detected.
*/
void handleAcceleration()
{
  mpu.getEvent(&a, &g, &temp);
  float acc_x = a.acceleration.x;
  float acc_y = a.acceleration.y;
  float acc_z = a.acceleration.z;
  float netAcc = abs(sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z) - 9.8);

  unsigned long fall_start = 0;
  ;
  bool fallStarted = false;
  unsigned int fall_duration = 0;
  while (netAcc > 8.5)
  {
    if (fallStarted == false)
    {
      Serial.println("Registered fall");
      fall_start = millis();
      fallStarted = true;
    }
    mpu.getEvent(&a, &g, &temp);
    acc_x = a.acceleration.x;
    acc_y = a.acceleration.y;
    acc_z = a.acceleration.z;
    netAcc = abs(sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z) - 9.8);
    delay(5);
  }
  if (fallStarted == true)
  {
    fall_duration = millis() - fall_start;
    fall_detected = true;
    Serial.print("Fall lasted for: ");
    Serial.print(fall_duration);
    Serial.println("ms");
    unsigned int height_above_base = curr_height - base_height;
    if (height_above_base < 0)
    {
      Serial.println("Height was negative... Skipping...");
      return;
    }
    float expected_duration = sqrt(2 * (height_above_base) / 10) * 1000;
    Serial.print("Calculated (using altitude) duration of fall: ");
    Serial.println(expected_duration);
    if (curr_height - base_height < 5 || fall_duration < 3 * expected_duration / 4)
    {
      send_json("false");
      Serial.println("Height and/or fall duration below threshold of 5 meters and 500ms respectively. Skipping...");
      return;
    }
    else
    {
      send_json(String("true"));
      Serial.print("Fall duration exceeded threshold duration.");
      Serial.println("ms");
      sendFallToThingspeak(fall_duration);
    }
  }
}

/*!
 * Send Data to thingspeak.
 * @param   bpm   Current bpm that is to be sent to thingspeak
 */
void sendToThingSpeak(int bpm)
{
  if (millis() - last_update_thingspeak_bpm > 60000)
  {
    Serial.println("Uploading data to thingspeak");
    ThingSpeak.setField(1, bpm);
    ThingSpeak.setField(2, bpm_avg);
    String bpm_status;
    if (bpm < 30 || bpm > 220)
    {
      bpm_status = String("offline");
      Serial.println("Going to sleep for 300 seconds");
      digitalWrite(LED2, HIGH);
      ESP.deepSleep(180000);
      digitalWrite(D0, HIGH);
    }
    else
    {
      bpm_status = String("online");
    }
    ThingSpeak.setStatus(bpm_status);

    // write to the ThingSpeak channel
    int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey); //Status Code From ThingSpeak
    if (x == 200)
    {
      Serial.println("Channel update successful.");
    }
    else
    {
      Serial.println("Problem updating channel. HTTP error code " + String(x));
    }
    last_update_thingspeak_bpm = millis();
  }
}

/*!
 *Sends only fall details to thingspeak.
 *@param fall_dur  Duration of fall recorded
 */
void sendFallToThingspeak(unsigned long fall_dur)
{
  if (fall_detected != true)
  {
    return;
  }
  Serial.println("Uploading accelaration to thingspeak");
  ThingSpeak.setField(4, (int)fall_dur);

  // write to the ThingSpeak channel
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if (x == 200)
  {
    Serial.println("Channel update successful.");
  }
  else
  {
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }
  last_update_thingspeak_acc = millis();
}

/*!
 *Set Threshold for calculating BPM. 
 *@param Signal   Current value of analogue signal from Pulse Sensor
*/
void setThresholdBPM(int Signal)
{

  if (Signal > pulse_max)
  {
    pulse_max = Signal;
  }
  else if (Signal < pulse_min)
  {
    pulse_min = Signal;
  }

  if (millis() - last_update_threshold > 2000)
  {
    ThresholdBPM = (pulse_min / 2) + (1 * pulse_max / 2);
    last_update_threshold = millis();
    pulse_min = ThresholdBPM;
    pulse_max = ThresholdBPM;
  }
}

/*!
 *Initialize MPU6050 with the fixed properties
 * @param   mpu   object of Adafruit_MPU6050
*/
void setMPUProperties(Adafruit_MPU6050 &mpu)
{
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange())
  {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange())
  {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth())
  {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
}

/*!
 *Send json data to Local django webserver.
 *@param  fall_occured  pass "true" if fall has occured, else "false". 
*/
void send_json(String fall_occurred)
{
  if (fall_occurred != "true" && fall_occurred != "false")
  {
    return;
  }
  http_client.begin(client, localServerName);
  Serial.println("");

  Serial.println("Sending data to 192.168.0.100/send");
  http_client.addHeader("Content-Type", "application/json");

  String post_json = String("{\n\t\"addr\":") + address + String(",\n\t\"data\": {\n\t\t\"fall_detected\":") + fall_occurred + String(",\n\t\t\"pulse\": {\n\t\t\t\"curr\":") + String(bpm_curr) + String(",\n\t\t\t\"avg\": ") + String(bpm_avg) + String("\n\t\t}, \n\t\t\"height\":") + String(curr_height - base_height) + String("\n\t\t}\n}");
  Serial.println(post_json);
  int httpResponseCode = http_client.POST(post_json);
  Serial.printf("[HTTP] Response Code: %d\n", httpResponseCode);
  http_client.end();
}
