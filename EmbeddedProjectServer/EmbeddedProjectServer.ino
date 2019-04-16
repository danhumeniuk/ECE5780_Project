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

const char WiFiPassword[] = "12345678";
const char AP_NameChar[] = "GyroData";
int8_t gyroData;

IPAddress apIP(192, 168, 4, 1);

ESP8266WebServer server(80);
DNSServer dnsServer;

String request = "";

void setup()
{
  ESP.eraseConfig();

  pinMode(LED_Pin, OUTPUT);

  boolean conn = WiFi.softAP(AP_NameChar, WiFiPassword, 6, 0);

  // Add handler function to server for home page
  server.on("/", dataHandler);

  server.begin();

  Serial.begin(115200);

  // DNS setup
  dnsServer.setTTL(300);
  dnsServer.setErrorReplyCode(DNSReplyCode::ServerFailure);
  dnsServer.start(53, "www.gyrodata.com", apIP);

}

// Main handler, responds to the '/' index
void dataHandler()
{
  String gyroDataString = String(gyroData);
  server.send(200, "text/plain", gyroDataString);
}

void getGyroData() {
  if (Serial.available() > 0) {
    gyroData = Serial.read();
  }
}

// Continously attempts to handle DNS or HTTP requestss
void loop()
{
  getGyroData();
  dnsServer.processNextRequest();
  server.handleClient();
  delay(50);
}
