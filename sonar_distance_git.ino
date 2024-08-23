#include <Arduino.h>
#include <WiFiNINA.h>
#include <utility/wifi_drv.h>
#include <ArduinoMqttClient.h> // MQTT-Protokoll
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MPU9250_WE.h>
#include <SPI.h> // SPI-Protokoll
#include <Wire.h> // I2C-Protokoll
#include "arduino_secrets.h"

// globale Konstanten
#define MPU9250_ADDR 0x68 //  -> uses Wire / MPU9250_ADDR
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

#define BME_SCK 9
#define BME_MISO 10
#define BME_MOSI 8
#define BME_CS 0  // Chip Select pin

#define TRIG_PIN 7 // TRIG pin
#define ECHO_PIN 6 // ECHO pin

//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI
Adafruit_BME280 bme(BME_CS); // hardware SPI

const int DISTANCE_THRESHOLD1 = 4; // centimeters
const int DISTANCE_THRESHOLD2 = 10; // centimeters

char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

const char broker[] = "domipi";
const int port = 1883;
const char topic_dist[]  = "/home/distance";
const char topic_gx[]  = "/home/gx";
const char topic_gy[]  = "/home/gy";
const char topic_gz[]  = "/home/gz";
const char topic_anglex[]  = "/home/anglex";
const char topic_angley[]  = "/home/angley";
const char topic_orient[]  = "/home/orient";

const int sensorPin = 1; // Sensor OUT-Pin verbunden mit digitalem Pin 2

// globale Variablen
int count = 0;
float distance; // store the distance from sensor

float gx = 0.00;
float gy = 0.00;
float gz = 0.00;
float anglex  = -0.07; 
float angley  = -0.02;
float anglez  = 90.00;
char orientation;


// float myArray[11] = {0.00, 1.00, 2.00, 3.00, 4.00, 5.00, 6.00, 7.00, 8.00, 9.00, 10.00};
float myArray[11] = {distance, gx, gy, gz, anglex, angley, anglez, bme.readTemperature(), bme.readPressure() / 100.0F, bme.readHumidity(), 10.00};



WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);


void wifi_mqtt_setup() {
    
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

void bme_setup() {
  while (!Serial) delay(100);   // Wait for serial console to open

  Serial.println("BME280 SPI Test");

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  Serial.println("BME280 sensor found!");
}


void gyro_setup() {
  Wire.begin();
  if(!myMPU9250.init()){
    Serial.println("MPU9250 does not respond");
  }
  else{
    Serial.println("MPU9250 is connected");
  }

  /* The slope of the curve of acceleration vs measured values fits quite well to the theoretical 
   * values, e.g. 16384 units/g in the +/- 2g range. But the starting point, if you position the 
   * MPU9250 flat, is not necessarily 0g/0g/1g for x/y/z. The autoOffset function measures offset 
   * values. It assumes your MPU9250 is positioned flat with its x,y-plane. The more you deviate 
   * from this, the less accurate will be your results.
   * The function also measures the offset of the gyroscope data. The gyroscope offset does not   
   * depend on the positioning.
   * This function needs to be called at the beginning since it can overwrite your settings!
   */
  Serial.println("Position you MPU9250 flat and don't move it - calibrating...");
  delay(1000);
  myMPU9250.autoOffsets();
  Serial.println("Done!");
  
  /*  This is a more accurate method for calibration. You have to determine the minimum and maximum 
   *  raw acceleration values of the axes determined in the range +/- 2 g. 
   *  You call the function as follows: setAccOffsets(xMin,xMax,yMin,yMax,zMin,zMax);
   *  Use either autoOffset or setAccOffsets, not both.
   */
  //myMPU9250.setAccOffsets(-14240.0, 18220.0, -17280.0, 15590.0, -20930.0, 12080.0);

  /*  Sample rate divider divides the output rate of the gyroscope and accelerometer.
   *  Sample rate = Internal sample rate / (1 + divider) 
   *  It can only be applied if the corresponding DLPF is enabled and 0<DLPF<7!
   *  Divider is a number 0...255
   */
  //myMPU9250.setSampleRateDivider(5);
  
  /*  MPU9250_ACC_RANGE_2G      2 g   
   *  MPU9250_ACC_RANGE_4G      4 g
   *  MPU9250_ACC_RANGE_8G      8 g   
   *  MPU9250_ACC_RANGE_16G    16 g
   */
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);

  /*  Enable/disable the digital low pass filter for the accelerometer 
   *  If disabled the bandwidth is 1.13 kHz, delay is 0.75 ms, output rate is 4 kHz
   */
  myMPU9250.enableAccDLPF(true);

 /*  Digital low pass filter (DLPF) for the accelerometer (if DLPF enabled) 
   *  MPU9250_DPLF_0, MPU9250_DPLF_2, ...... MPU9250_DPLF_7 
   *   DLPF     Bandwidth [Hz]      Delay [ms]    Output rate [kHz]
   *     0           460               1.94           1
   *     1           184               5.80           1
   *     2            92               7.80           1
   *     3            41              11.80           1
   *     4            20              19.80           1
   *     5            10              35.70           1
   *     6             5              66.96           1
   *     7           460               1.94           1
   */
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);  
}

void setup() {
  // begin serial port
  // Initialize serial and wait for port to open:
  Serial.begin(115200);

  // configure the trigger and echo pins to output mode
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(sensorPin, INPUT);

  WiFiDrv::pinMode(25, OUTPUT); //define green pin
  WiFiDrv::pinMode(26, OUTPUT); //define red pin
  WiFiDrv::pinMode(27, OUTPUT); //define blue pin
  
  wifi_mqtt_setup();
  gyro_setup();
  bme_setup();
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
  Serial.println("---------------------------------------------------------------");

  xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat angle = myMPU9250.getAngles();
  /* For g-values the corrected raws are used */
  Serial.print("g-x      = ");
  Serial.print(gValue.x);
  Serial.print("  |  g-y      = ");
  Serial.print(gValue.y);
  Serial.print("  |  g-z      = ");
  Serial.println(gValue.z);

  /* Angles are also based on the corrected raws. Angles are simply calculated by
    angle = arcsin(g Value) */
  Serial.print("Angle x  = ");
  Serial.print(angle.x);
  Serial.print("  |  Angle y  = ");
  Serial.print(angle.y);
  Serial.print("  |  Angle z  = ");
  Serial.println(angle.z);

  Serial.print("Orientation of the module: ");
  Serial.println(myMPU9250.getOrientationAsString());

  if (distance_cm < DISTANCE_THRESHOLD1) {
  WiFiDrv::analogWrite(25, 0);
  WiFiDrv::analogWrite(26, 255);
  WiFiDrv::analogWrite(27, 0);
  } 
  else if (distance_cm >= DISTANCE_THRESHOLD1 && distance_cm < DISTANCE_THRESHOLD2) {
  WiFiDrv::analogWrite(25, 255);
  WiFiDrv::analogWrite(26, 100);
  WiFiDrv::analogWrite(27, 0);
  }
  else {
  WiFiDrv::analogWrite(25, 255);
  WiFiDrv::analogWrite(26, 0);
  WiFiDrv::analogWrite(27, 0);
  }

  mqttClient.beginMessage(topic_dist);
  mqttClient.print(distance_cm);
  mqttClient.endMessage();
  mqttClient.beginMessage(topic_gx);
  mqttClient.print(gValue.x);
  mqttClient.endMessage();

  Serial.println("---------------------------------------------------------------");
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" °C");

  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();

  int sensorValue = digitalRead(sensorPin);
  
  // Ausgabe des Sensorwertes auf die serielle Schnittstelle
  Serial.println(sensorValue);

  // delay(500);
  delay(1000);
  Serial.println();

}
