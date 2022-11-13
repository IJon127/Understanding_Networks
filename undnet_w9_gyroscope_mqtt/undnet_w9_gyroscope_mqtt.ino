/**************************************************************************
  Gyroscope data via mqtt
  Uses Arduino LSM6DS3 library example (by Riccardo Rizzo): https://www.arduino.cc/en/Reference/ArduinoLSM6DS3
  Uses Tom Igoe's MqttClientButtonLed: https://github.com/tigoe/mqtt-examples/blob/main/MqttClientButtonLed/MqttClientButtonLed.ino
  created 13 Nov 2022
  modified 13 Nov 2022
  by I-Jon Hsieh
 **************************************************************************/

#include <Arduino_LSM6DS3.h>
#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>
#include "secret.h"

//initialize wifi connection:
WiFiClient wifi;
MqttClient mqttClient(wifi);

//detials for MQTT client:
char broker[] = "public.cloud.shiftr.io";
int port = 1883;
char topic[] = "itpGyroscope";
char clientID[] = "sensorClient";

// connection led indicator
const int wifiLedPin = 2; //white LED
const int brokerLedPin = 3; //blue LED

// last states
float lastX = 0;
float lastY = 0;
float lastZ = 0;

//threshole
int angleThreshold = 5;



void setup() {
  //set pins
  pinMode(wifiLedPin, OUTPUT);  
  pinMode(brokerLedPin, OUTPUT); 
  digitalWrite(wifiLedPin, LOW);
  digitalWrite(brokerLedPin, LOW);



  Serial.begin(9600);
  //while (!Serial);

  // MQTT setup and connect
  mqttClient.setId(clientID);
  mqttClient.setUsernamePassword(SECRET_MQTT_USER, SECRET_MQTT_PASS);
  connectToBroker();


  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");
}

void loop() {
  float x, y, z;

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);

    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);

    lastX = x;
    lastY = y;
    lastZ = z;
  }

  //MQTT connection:
  if (!mqttClient.connected()){    //if not connectd to the broker, try to connect:
    Serial.println("reconnecting");
    connectToBroker();
  }

  //Sent xyz data to MQTT broker:
  
  if (abs(x)>angleThreshold || abs(y)>angleThreshold || abs(z)>angleThreshold) {
      Serial.println("send!");
      mqttClient.beginMessage(topic);
      mqttClient.print(x);
      mqttClient.print('\t');
      mqttClient.print(y);
      mqttClient.print('\t');
      mqttClient.println(z);
      mqttClient.endMessage();
  } 
}





// connection functions ----------------------------

boolean connectToBroker(){
  checkWifiConnection();
  
  //if MQtt client is not connected:
  if (!mqttClient.connect(broker, port)){
    Serial.print("MQTT connection failed. Error no: ");
    Serial.println(mqttClient.connectError());
    digitalWrite(brokerLedPin, LOW);
    
    return false;
  }

  //once you're connected, process...
  mqttClient.subscribe(topic);
  digitalWrite(brokerLedPin, HIGH);


  return true;
}


void checkWifiConnection(){
    while (WiFi.status() != WL_CONNECTED) {
    Serial.print("Attempting to connecting to ");
    Serial.println(SECRET_SSID);
    WiFi.begin(SECRET_SSID, SECRET_PASS);
    digitalWrite(wifiLedPin, HIGH);
    delay(500);
    digitalWrite(wifiLedPin, LOW);
    delay(500);
    digitalWrite(wifiLedPin, HIGH);
    delay(500);
    digitalWrite(wifiLedPin, LOW);
    delay(500);
  }

  Serial.print("Connected. My IP address: ");
  Serial.println(WiFi.localIP());
  digitalWrite(wifiLedPin, HIGH);
}
