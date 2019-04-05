/*
 * Sketch: led_test
 * Control an LED from a web browser
 * Intended to be run on an ESP8266
 * Go to led.gov after connecting to LEDControl
 * Password is 12345678
 */
 
 
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h> 
#include <ESP8266WebServer.h>
#include <DNSServer.h>     

const char WiFiPassword[] = "12345678";
const char AP_NameChar[] = "LEDControl" ;

IPAddress apIP(192, 168, 4, 1);

ESP8266WebServer server(80);
DNSServer dnsServer;

// Our webpage variables
// html_2 is used to change the button text
String html_1 = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'/><meta charset='utf-8'><style>body {font-size:140%;} #main {display: table; margin: auto;  padding: 0 10px 0 10px; } h2,{text-align:center; } .button { padding:10px 10px 10px 10px; width:100%;  background-color: #4CAF50; font-size: 120%;}</style><title>LED Control</title></head><body><div id='main'><h2>LED Control</h2>";
String html_2 = "";
String html_3 = "</div></body></html>";
 
String request = "";
int LED_Pin = 0;
 
void setup() 
{
    ESP.eraseConfig();
  
    pinMode(LED_Pin, OUTPUT); 
 
    boolean conn = WiFi.softAP(AP_NameChar, WiFiPassword, 6, 0);
   
    // Add handler function to server for home page
    server.on("/", httpHandler);
    server.on("/control", commandHandler); // Command handler 
    
    server.begin();

    Serial.begin(115200);
    Serial.println();

    if (conn)
      Serial.println("AP established!");
    else
      Serial.println("Failed!");

    // DNS setup
    dnsServer.setTTL(300);
    dnsServer.setErrorReplyCode(DNSReplyCode::ServerFailure);
    dnsServer.start(53, "www.led.gov", apIP);
    
}

// Responds to '/control' with parameter 'command'
void commandHandler()
{
  request = server.arg("command");
  if       ( request == "ON")  { digitalWrite(LED_Pin, LOW);  }
  else if  ( request == "OFF") { digitalWrite(LED_Pin, HIGH);   }

  setButton();
  server.send(200, "text/html", html_1 + html_2 + html_3 );
}

// Main handler, responds to the '/' index
void httpHandler() 
{ 
    setButton();
    server.send(200, "text/html", html_1 + html_2 + html_3 );
}

// Sets the button to correctly show on or off
void setButton()
{
    // Get the LED pin status and create the LED status message
    if (digitalRead(LED_Pin) == LOW) 
    {
        // the LED is on so the button needs to say turn it off
       html_2 = "<form id='F1' action='control'><input class='button' type='submit' value='Turn off the LED' /><input type='hidden' name='command' value='OFF' /></form><br>";
    }
    else                              
    {
        // the LED is off so the button needs to say turn it on
       html_2 = "<form id='F1' action='control'><input class='button' type='submit' value='Turn on the LED' /><input type='hidden' name='command' value='ON' /></form><br>";
    }  
}

// Continously attempts to handle DNS or HTTP requestss
void loop() 
{
    dnsServer.processNextRequest();
    server.handleClient();
    delay(50); 
}
