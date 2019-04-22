/*
   The webserver for the embedded project. 
   Will be run on the esp8266.
   Access point is GyroData and the password 
   is 12345678.
*/


#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <WiFiUdp.h>

const char WiFiPassword[] = "12345678";
const char AP_NameChar[] = "GyroData";
int8_t gyroData;

IPAddress apIP(192, 168, 4, 1);

ESP8266WebServer server(80);
DNSServer dnsServer;
WiFiUDP Udp;

char incomingPacket[255];
int udpPort = 5000;

String request = "";

void setup()
{
  ESP.eraseConfig();

  boolean conn = WiFi.softAP(AP_NameChar, WiFiPassword, 6, 0);

  // Add handler function to server for home page
  server.on("/", indexHandler);
  server.on("/data", dataHandler);

  server.begin();

  Serial.begin(115200);

  // DNS setup
  dnsServer.setTTL(300);
  dnsServer.setErrorReplyCode(DNSReplyCode::ServerFailure);
  dnsServer.start(53, "www.gyrodata.com", apIP);

  // Port 5000 for testing
  Udp.begin(udpPort);
}

void processUDP() {
  int packetSize = Udp.parsePacket();

  // Check if there is data to parse
  if (packetSize)
  {
    Udp.read(incomingPacket, 255);

    Serial.printf("Received packet: %d ", incomingPacket[0]);

    // If the packet starts with a 1, send the rotation
    if (incomingPacket[0] == 1)
    {
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(gyroData);
      Udp.endPacket();
      Serial.printf("Sent packet: %d\n", gyroData);
    }
  }
}

void indexHandler() {
   server.send(200, "text/plain", "Connection Success :)");
}

// Main handler, responds to the '/data' index
void dataHandler()
{
  String gyroDataString = String(gyroData);
  Serial.println("String representation: " + gyroDataString);
  server.send(200, "text/plain", gyroDataString);
}

void getGyroData() {
  // Flush the serial buffer to get the updated gyroscope location
  while (Serial.available() > 0) {
    gyroData = Serial.read();
  }
}

// Continously attempts to handle DNS or HTTP requestss
void loop()
{
  getGyroData();
  dnsServer.processNextRequest();
  server.handleClient();
  processUDP();
  delay(25);
}
