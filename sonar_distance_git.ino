/*
 * Created by ArduinoGetStarted.com
 *
 * domiArduino.ino
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-ultrasonic-sensor
 * This example code is in the public domain.
 * https://www.arduino.cc  
*/

#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h"

// globale Variablen
int led = 5;
#define TRIG_PIN 7 // TRIG pin
#define ECHO_PIN 6 // ECHO pin

float filterArray[20]; // array to store data samples from sensor
float distance; // store the distance from sensor
//float clean_distance = 0.00;

char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

const char broker[] = "domipi";
int        port     = 1883;
const char topic[]  = "/home/distance/";

int count = 0;

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);


void mqtt_setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  
  // wait for serial port to connect. Needed for native USB port only
  while (!Serial) {   
    ; 
  }
  // attempt to connect to Wifi network:
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);

  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }

  Serial.println("You're connected to the network! :-)\n");
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    while (1);
  }

  Serial.println("You're connected to the MQTT broker! :-D\n");
}


void setup() {
  // begin serial port
  Serial.begin (9600);

  // configure the trigger and echo pins to output mode
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  mqtt_setup();
}

void loop() {
  // call poll() regularly to allow the library to send MQTT keep alive which
  // avoids being disconnected by the broker
  mqttClient.poll();

  // generate 10-microsecond pulse to TRIG pin
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // measure duration of pulse from ECHO pin
  float duration_us = pulseIn(ECHO_PIN, HIGH);

  // calculate the distance
  float distance_cm = 0.017 * duration_us;

  // print the value to Serial Monitor
  Serial.print("distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");

  mqttClient.beginMessage(topic);
  mqttClient.print(distance_cm);
  mqttClient.endMessage();

  delay(500);

}
