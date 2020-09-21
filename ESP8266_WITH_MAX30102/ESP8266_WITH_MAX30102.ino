#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

//VARIABEL SENSOR
MAX30105 particleSensor;
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;

//VARIABEL MQTT
const char* ssid = "honor 9";
const char* password = "sandisayaaaa";
const char* mqttServerIP = "broker.hivemq.com";
const int mqttPort = 1883;
WiFiClient espClient;
PubSubClient client(espClient);

//define topic
char* Topic1 = "Imedic/Data1";
char* Topic2 = "Imedic/Data2";// publish

void reconnect(){
  // MQTT Begin
  while(!client.connected()){
    Serial.println("Connecting to MQTT Server..");
    Serial.print("IP MQTT Server : "); Serial.println(mqttServerIP);

    bool hasConnection = client.connect("kans");
    if(hasConnection){
      Serial.println("Success connected to MQTT Broker");
    } else {
      Serial.print("Failed connected");
      Serial.println(client.state());
      digitalWrite(4,HIGH);
      delay(2000);
      Serial.println("Try to connect...");
    }
  }
  client.publish(Topic1, "Reconnecting");
  client.publish(Topic2, "Reconnecting");
}

void callback(char* topic, byte* payload, unsigned int length){
  Serial.println("--------");
  Serial.println("Message Arrived");
  Serial.print("Topic :"); Serial.println(topic);
  Serial.print("Message : ");
  String pesan = "";
  for(int i=0; i < length; i++){
    Serial.print((char)payload[i]);
    pesan += (char)payload[i];
  }
  if(pesan == "ON" ){
    Serial.println("LED ON.. Warning");
    digitalWrite(4,HIGH);
  } else if(pesan == "OFF"){
    Serial.println("LED OFF.. Safety");
    digitalWrite(4,LOW);
  }
  Serial.println("ESP/CMD topic arrived");
  Serial.println(pesan);
  Serial.print("Pesan masuk :");
  Serial.println(pesan);
  Serial.println("--------");
}

void setup() {
  Serial.begin(57600);
  pinMode(4, OUTPUT);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected - ESP IP address: ");
  Serial.println(WiFi.localIP());

  // Initialize sensor
  Serial.println("Initializing...");
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  client.setServer(mqttServerIP, mqttPort);
  client.setCallback(callback);
  delay(20);
}

char dataPublish[50];
void publishMQTT(char* topics, String data){
   data.toCharArray(dataPublish, data.length() + 1);
   client.publish(topics, dataPublish);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  if(!client.loop()){
    client.connect("ESP8266Client");
  }
  
  long irValue = particleSensor.getIR();
  
  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0);
    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable
      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
  
  publishMQTT(Topic1,String(irValue));
  publishMQTT(Topic2,String(beatsPerMinute));
  
  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);
  if (irValue < 20000){
    Serial.print(" [No finger!]");
  }
  Serial.print(" [MQTT Running!]");
  Serial.println();

  delay(100);
}
