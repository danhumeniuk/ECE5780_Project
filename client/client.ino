/*
    HTTP over TLS (HTTPS) example sketch
    This example demonstrates how to use
    WiFiClientSecure class to access HTTPS API.
    We fetch and display the status of
    esp8266/Arduino project continuous integration
    build.
    Limitations:
      only RSA certificates
      no support of Perfect Forward Secrecy (PFS)
      TLSv1.2 is supported since version 2.4.0-rc1
    Created by Ivan Grokhotkov, 2015.
    This example is in public domain.
*/

#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>

#ifndef STASSIpD
#define STASSID "GyroData"
#define STAPSK  "12345678"
#endif

const char* ssid = STASSID;
const char* password = STAPSK;

const char* host = "gyrodata.com";
const int httpsPort = 80;
volatile int gyroData = 0;
int in1Pin = 12;
int in2Pin = 13;
int motorSpeed = 255;

char incomingPacket[255];

// Use WiFiClientSecure class to create TLS connection
WiFiClient client;
WiFiUDP Udp;

IPAddress serverIP(192, 168, 4, 1);
int udpPort = 5000;

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.print("connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  Udp.begin(udpPort);
}

void getGyroData() { 
  Serial.print("connecting to ");
  Serial.println(host);


  if (!client.connect(host, httpsPort)) {
    Serial.println("connection failed");
    return;
  }
  
  String url = "/data";
  Serial.print("requesting URL: ");
  Serial.println(url);

  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "User-Agent: BuildFailureDetectorESP8266\r\n" +
               "Connection: close\r\n\r\n");

  Serial.println("request sent");
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") {
      Serial.println("headers received");
      break;
    }
  }
  
  String line = client.readStringUntil('\n');
  
  Serial.println("reply was:");
  Serial.println("==========");
  Serial.println(line);
  Serial.println("==========");
  Serial.println("closing connection");
  if (line.charAt(0) == '-') {
    gyroData = line.substring(1).toInt();
    gyroData = gyroData * -1;
  } else {
    gyroData = line.toInt();
  }
  Serial.println(gyroData);
}

// Uses UDP to get the gyroscope data
void getGyroDataUDP() {
  // Constantly send the server a 1 (server port is 5000)
  Udp.beginPacket(serverIP, udpPort);
  Udp.write(1);
  if (!Udp.endPacket())
    Serial.println("Error sending UDP packet!");

  // Check for incoming packets
  int packetSize = Udp.parsePacket();

  // Check if there is data to parse
  if (packetSize)
  {
    Udp.read(incomingPacket, 255);
    gyroData = incomingPacket[0];
    if (gyroData > 127) // Check if negative
      gyroData = gyroData - 256;
    Serial.println("Got gyro data: " + String(gyroData));
  }
}

void loop() {
  //getGyroData();
  getGyroDataUDP();
  if (gyroData > 15) {
    analogWrite(in1Pin, motorSpeed);
    analogWrite(in2Pin, 0);
  } else if (gyroData < -15) {
    analogWrite(in1Pin, 0);
    analogWrite(in2Pin, motorSpeed);
  } else {
    analogWrite(in1Pin, 0);
    analogWrite(in2Pin, 0);
  }
  delay(50);
}
