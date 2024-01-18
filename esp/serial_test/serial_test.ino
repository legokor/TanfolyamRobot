
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>

#ifndef APSSID
#define APSSID "ESP_test"
#define APPSK "sendhelp"
#endif


const char *ssid = APSSID;
const char *password = APPSK;

ESP8266WebServer server(80);

const int bufferSize = 128;  // Adjust the buffer size as needed
char serialBuffer[bufferSize] = {};
char htmlMessage[bufferSize] = {};
int bufferIndex = 0;

enum State {
  IDLE,
  READING
};

State currentState = IDLE;

void handleRoot() {
  server.send(200, "text/html", htmlMessage);
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Serial started");
  
  WiFi.softAP(ssid, password);

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.on("/", handleRoot);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
  if (Serial.available() > 0) {
    char incomingChar = Serial.read();

    switch (currentState) {
      case IDLE:
        if (incomingChar == 'D') {
          currentState = READING;
          bufferIndex = 0;
        }
        break;

      case READING:
        if (incomingChar == '\n') {
          serialBuffer[bufferIndex] = '\0';  // Null-terminate the string
          processSerialData(serialBuffer);
          currentState = IDLE;
        } else {
            serialBuffer[bufferIndex] = incomingChar;
            bufferIndex++;
        }
        break;
    }
  }
}

void processSerialData(const char *data) {
  //Serial.print("Received Data: ");
  //Serial.println(data);
  strcpy(htmlMessage, data);
}
