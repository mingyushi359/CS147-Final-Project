#include <Arduino.h>
#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include "Wire.h"
#include <ArduinoJson.h>

// Wi-Fi credentials
#define WIFI_SSID ""
#define WIFI_PASSWORD ""

// Azure IoT Hub configuration
#define SAS_TOKEN ""
// Root CA certificate for Azure IoT Hub
const char* root_ca = "";

String iothubName = "";
String deviceName = "";
String url = "https://" + iothubName + ".azure-devices.net/devices/" + 
             deviceName + "/messages/events?api-version=2021-04-12";

// Telemetry interval
#define TELEMETRY_INTERVAL 5000 // Send data every 5 seconds
unsigned long last_gps_sent_time;

const int RXPin = 22, TXPin = 21;
const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
HardwareSerial GPSSerial(1);
// SoftwareSerial ss(RXPin, TXPin);

const int sensor1_trig_pin = 27;
const int sensor1_echo_pin = 26;
const int sensor2_trig_pin = 13;
const int sensor2_echo_pin = 17;
bool use_sensor1 = true;

const int green = 32;
const int yellow = 33;
const int red = 25;
const int green_threshold = 75; // cm
const int yellow_threshold = 50;
const int red_threshold = 25;

const int buzzer = 12;
const int buzzer_switch = 0;

unsigned long last_beep_time;
const int beep_length = 100; // ms
unsigned long beep_interval = 1000;
bool beeping = false;

const int average_reading_count = 1;  // take average of last few readings, currently set to 1, no effect
long sum_reading = 0.0;
int current_reading_count = 0;

void TaskDistanceMeasure(void *pvParameters);
void TaskGpsUpload(void *pvParameters);

void change_led_state(int distance);
void change_buzzer_frequency(int distance);
void displayInfo();
void send_gps_data();

void setup() {
  Serial.begin(9600);
  GPSSerial.begin(9600, SERIAL_8N1, RXPin, TXPin);
  // ss.begin(GPSBaud);
  Wire.begin();

  xTaskCreatePinnedToCore(  // create freertos task 
    TaskDistanceMeasure,
    "TaskDistanceMeasure",
    4096,   // stack size
    NULL,   // parameters to pass
    3,      // task priority
    NULL,
    1       // core to run
  );

  xTaskCreatePinnedToCore(
    TaskGpsUpload,
    "TaskGpsUpload",
    8192,   // stack size
    NULL,   // parameters to pass
    1,      // task priority
    NULL,
    0       // core to run
  );

  // vTaskStartScheduler();
}

void loop() {
}

void TaskDistanceMeasure(void *pvParameters) {  // Task for measuring distance and controls buzzer/LEDs 
  pinMode(sensor1_trig_pin, OUTPUT);  // ultrasonic sensor
  pinMode(sensor1_echo_pin, INPUT);
  pinMode(sensor2_trig_pin, OUTPUT);
  pinMode(sensor2_echo_pin, INPUT);

  pinMode(green, OUTPUT);  // LEDs
  pinMode(yellow, OUTPUT);
  pinMode(red, OUTPUT);

  pinMode(buzzer, OUTPUT);  // buzzer
  pinMode(buzzer_switch, INPUT_PULLUP);

  // Ensure trigger pin is low
  digitalWrite(sensor1_trig_pin, LOW);
  digitalWrite(sensor2_trig_pin, LOW);
  vTaskDelay(pdMS_TO_TICKS(2));

  unsigned long last_print_time = millis();

  while(true) {
    // Send a 10 microsecond high pulse to trigger the sensor
    // alternate two sensors to avoid conflict signals
    long duration = 0.0;
    if (use_sensor1) {
      digitalWrite(sensor1_trig_pin, HIGH);
      vTaskDelay(pdMS_TO_TICKS(10));
      digitalWrite(sensor1_trig_pin, LOW);

      duration = pulseIn(sensor1_echo_pin, HIGH);  // read the duration of the pulse in ms
    } else {
      digitalWrite(sensor2_trig_pin, HIGH);
      vTaskDelay(pdMS_TO_TICKS(10));
      digitalWrite(sensor2_trig_pin, LOW);

      duration = pulseIn(sensor2_echo_pin, HIGH);
    }

    // Calculate distance in centimetres
    int distance = duration * 0.034 / 2;

    // Print the distance in cm
    if (millis() - last_print_time >= 1050) {
      if (use_sensor1) {
        Serial.print("Sensor 1: ");
      } else {
        Serial.print("Sensor 2: ");
      }
      Serial.print(distance);
      Serial.println(" cm");
      
      last_print_time = millis();
    }

    sum_reading += distance;
    current_reading_count++;
    if (current_reading_count >= average_reading_count) {  // calculate the average reading
      distance = sum_reading / average_reading_count;
      sum_reading = 0.0;
      current_reading_count = 0;

      // Serial.print("Averaged Distance: ");
      // Serial.print(distance);
      // Serial.println(" cm");

      change_led_state(distance);
      change_buzzer_frequency(distance);
    }

    use_sensor1 = !use_sensor1;  // alternate sensor state
    vTaskDelay(pdMS_TO_TICKS(50));
  }

}
void TaskGpsUpload(void *pvParameters) {  // Task for reading and uploading GPS data
  last_gps_sent_time = millis();

  WiFi.mode(WIFI_STA);  // initialize wifi
  vTaskDelay(pdMS_TO_TICKS(1000));
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
    Serial.print(WiFi.status());
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("MAC address: ");
  Serial.println(WiFi.macAddress());

  unsigned long last_gps_print_time = millis();

  while(true) {
    for (int i = 0; i < 16 && GPSSerial.available(); ++i) { // check for GPS data
      if (gps.encode(GPSSerial.read())) {
        // if (millis() - last_gps_print_time >= 1000) {
        //   displayInfo();
        //   last_gps_print_time = millis();
        // }
        if (gps.location.isValid() && millis() - last_gps_sent_time >= TELEMETRY_INTERVAL) {
          send_gps_data();  // send gps data periodically only if there's valid gps data
        }
      }
    }

    if (millis() > 5000 && gps.charsProcessed() < 10) {
      Serial.println(F("No GPS detected: check wiring."));
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void change_led_state(int distance) {
  // change LED (Red, Yellow, Green) based on distance
  if (distance <= red_threshold) {
    digitalWrite(red, HIGH);
    digitalWrite(yellow, LOW);
    digitalWrite(green, LOW);
  } else if (distance <= yellow_threshold) {
    digitalWrite(red, LOW);
    digitalWrite(yellow, HIGH);
    digitalWrite(green, LOW);
  } else if (distance <= green_threshold) {
    digitalWrite(red, LOW);
    digitalWrite(yellow, LOW);
    digitalWrite(green, HIGH);
  } else {
    digitalWrite(red, LOW);
    digitalWrite(yellow, LOW);
    digitalWrite(green, LOW);
  }
}

void change_buzzer_frequency(int distance) {
  if (distance > green_threshold) {  // stop beeping if too far
    noTone(buzzer);
    beeping = false;
    return;
  }

  if (distance <= red_threshold) {
    tone(buzzer, 1000);
    return;
  }

  beep_interval = map(distance, green_threshold, red_threshold, 500, 100);  // map distance to beep interval
  unsigned long current_time = millis();

  if (current_time - last_beep_time >= beep_interval) {  // silent for invertal time, then turn on
      tone(buzzer, 1000, beep_length);
      beeping = true;
      last_beep_time = current_time;
  }

}

void displayInfo()  // code example from mikalhart/TinyGPSPlus
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  Serial.println();
}

void send_gps_data() {
  if (gps.location.lat() == 0.0 || gps.location.lng() == 0.0) {  // skip if gps isn't fully initialized
    return;
  }

  // example code copied from Lab4
  // Create JSON payload
  ArduinoJson::JsonDocument doc;
  // doc["latitude"] = 33.679470;
  // doc["longitude"] = -117.844667;
  doc["latitude"] = gps.location.lat();
  doc["longitude"] = gps.location.lng();
  char buffer[256];
  serializeJson(doc, buffer, sizeof(buffer));

  // Send telemetry via HTTPS
  WiFiClientSecure client;
  client.setCACert(root_ca); // Set root CA certificate
  HTTPClient http;
  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", SAS_TOKEN);
  int httpCode = http.POST(buffer);

  if (httpCode == 204) { // IoT Hub returns 204 No Content for successful telemetry
    Serial.println("Telemetry sent: " + String(buffer));
  } else {
    Serial.println("Failed to send telemetry. HTTP code: " + String(httpCode));
  }
  http.end();  

  last_gps_sent_time = millis();
}