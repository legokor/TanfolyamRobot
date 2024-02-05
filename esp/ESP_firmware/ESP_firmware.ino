#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include <ESP8266WebServer.h>
/*
const char* ssid = "Andi";
const char* password = "Gomboc02";
const char* ipaddr = "192.168.0.100";
*/

const int bufferSize = 128; 
char serialBuffer[bufferSize] = {};
char htmlMessage[bufferSize] = {};
int bufferIndex = 0;

char host[30] = "Host: ";
char ssid[bufferSize] = {};
char password[bufferSize] = {};
char ipaddr[16] = {};
char tmp_ssid[bufferSize] = {};
char tmp_password[bufferSize] = {};
char tmp_ipaddr[16] = {};

enum State {
  PRECONFIG,
  IDLE,
  READING_C1,
  READING_C2,
  READING_D1,
  READING_D2
};

State currentState = PRECONFIG;

ESP8266WebServer server(80); // Server on port 80

constexpr bool DEBUG_ENABLED = false;

void dlog(const String& str){
    if(DEBUG_ENABLED){
        Serial.print("DEBUG: ");
        Serial.println(str);
    }
}

void setup() {
  Serial.begin(115200);
  dlog("");
  dlog("Serial started");

  getConfig();

  sscanf(serialBuffer, "\t%s\t%s\t%s", ssid, password, ipaddr);
  strcat(host, ipaddr);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    dlog("Connecting to WiFi...");
  }

  dlog("Connected to WiFi");
  // Handle POST requests
  server.on("/receiveData", HTTP_POST, []() {
    if (server.hasArg("plain")) {
    String message = server.arg("plain");
    char msg[500];
    
    dlog("Received message: " + message);
    sscanf(message.c_str(), "{\"message\":\"%[^\"]\"}", msg);
    Serial.println(msg);
    }
    server.send(200, "text/plain", "Data received");
  });

  server.begin(); // Start the server
}

void loop() {
  server.handleClient();
  
  if (Serial.available() > 0) {
    char incomingChar = Serial.read();

    switch (currentState) {
      case IDLE:
        if (incomingChar == 'D') {
          currentState = READING_D1;
          bufferIndex = 0;
        }
        if (incomingChar == 'C') {
          currentState = READING_C1;
          bufferIndex = 0;
        }
        break;
        
      case READING_D1:
        if (incomingChar == ':') {
          currentState = READING_D2;
          bufferIndex = 0;
        }
        break;

      case READING_D2:
        if (incomingChar == '\n') {
          serialBuffer[bufferIndex] = '\0';  // Null-terminate the string
          processSerialData(serialBuffer);
          currentState = IDLE;
        } else {
            serialBuffer[bufferIndex] = incomingChar;
            bufferIndex++;
        }
        break;
     
        case READING_C1:
          if (incomingChar == ':') {
            currentState = READING_C2;
            bufferIndex = 0;
          }
          break;
  
        case READING_C2:
          if (incomingChar == '\n') {
            serialBuffer[bufferIndex] = '\0';  // Null-terminate the string
            currentState = IDLE;
            sscanf(serialBuffer, "\t%s\t%s\t%s", tmp_ssid, tmp_password, tmp_ipaddr);
            if(strcmp(tmp_ssid, ssid) != 0 || strcmp(tmp_password, password) != 0 || strcmp(tmp_ipaddr, ipaddr) != 0){
              strcpy(ssid, tmp_ssid);
              strcpy(password, tmp_password);
              strcpy(ipaddr, tmp_ipaddr);
              strcat(host, ipaddr);
              WiFi.begin(ssid, password);
              while (WiFi.status() != WL_CONNECTED) {
                delay(1000);
                dlog("Connecting to WiFi...");
              }
              dlog("Connected to WiFi");
            }
            break;
          } else {
              serialBuffer[bufferIndex] = incomingChar;
              bufferIndex++;
          }
          break;
    }
  }
}

void getConfig(){
    while(currentState != IDLE){        
      if (Serial.available() > 0) {
        char incomingChar = Serial.read();
    
        switch (currentState) {
          case PRECONFIG:
            if (incomingChar == 'C') {
              currentState = READING_C1;
              bufferIndex = 0;
            }
            break;
          case READING_C1:
            if (incomingChar == ':') {
              currentState = READING_C2;
              bufferIndex = 0;
            }
            break;
    
          case READING_C2:
            if (incomingChar == '\n') {
              serialBuffer[bufferIndex] = '\0';  // Null-terminate the string
              currentState = IDLE;
              break;
            } else {
                serialBuffer[bufferIndex] = incomingChar;
                bufferIndex++;
            }
            break;
        }
      }  
    }
}

void processSerialData(const char *data) {
  int servo, motora, motorb, usonic, hsv_h, hsv_s, hsv_v, rgb_r, rgb_g, rgb_b;
  float wheelaspeed, wheelbspeed, imu_acc_a, imu_acc_b, imu_acc_c, imu_gyro_x, imu_gyro_y, imu_gyro_z, imu_temp;
  
  sscanf(data, " %d %d %d %f %f %ld %ld %d %d", &rgb_r, &rgb_g, &rgb_b, &wheelaspeed, &wheelbspeed, &motora, &motorb, &servo, &usonic);
  rgb2hsv(rgb_r, rgb_g, rgb_b, &hsv_h, &hsv_s, &hsv_v);
  sendJson(servo, motora, motorb, wheelaspeed, wheelbspeed, usonic, hsv_h, hsv_s, hsv_v, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

void rgb2hsv(int r, int g, int b, int* hue, int* sat, int* val){
  int max, min;
  float hue_f;

  min = (r < g) ? r : g;
  max = (r > g) ? r : g;
  min = (min < b) ? min : b;
  max = (max > b) ? max : b;

  *val = (int)max * 100 / 255;

  int delta = max - min;

  if (delta == 0) {
    *sat = 0;
    *hue = 0;
  } else {
    *sat = 100 * delta / max;

    if (max == r) {
      hue_f = 60 * ( ( ((float)g - (float)b) / delta )    );    // TODO: optimize
    } else if (max == g) {
      hue_f = 60 * ( ( ((float)b - (float)r) / delta ) + 2);    // TODO: optimize
    } else {
      hue_f = 60 * ( ( ((float)r - (float)g) / delta ) + 4);    // TODO: optimize
    }
    if (hue_f < 0) {
      hue_f += 360;
    }
      *hue = (int)hue_f % 360;
  }
}

void sendJson(int servo, int motora, int motorb, float wheelaspeed, float wheelbspeed, int usonic, int hsv_h, int hsv_s, int hsv_v, float imu_acc_a, float imu_acc_b, float imu_acc_c, float imu_gyro_x, float imu_gyro_y, float imu_gyro_z, float imu_temp) {
    WiFiClient client;
    
    if (!client.connect(ipaddr, 5000)) {
        Serial.println("Connection failed");
        return;
    }
  
    StaticJsonDocument<256> doc;
    
    doc["servo"] = servo;
    doc["motora"] = motora;
    doc["motorb"] = motorb;
    doc["wheelaspeed"] = wheelaspeed;
    doc["wheelbspeed"] = wheelbspeed;
    doc["usonic"] = usonic;
    
    JsonObject hsv = doc.createNestedObject("hsv");
    hsv["h"] = hsv_h;
    hsv["s"] = hsv_s;
    hsv["v"] = hsv_v;
    
    JsonObject imu = doc.createNestedObject("imu");
    
    JsonObject imu_acc = imu.createNestedObject("acc");
    imu_acc["a"] = imu_acc_a;
    imu_acc["b"] = imu_acc_b;
    imu_acc["c"] = imu_acc_c;
    
    JsonObject imu_gyro = imu.createNestedObject("gyro");
    imu_gyro["x"] = imu_gyro_x;
    imu_gyro["y"] = imu_gyro_y;
    imu_gyro["z"] = imu_gyro_z;
    
    imu["temp"] = imu_temp;
    
    String jsonStr;
    serializeJson(doc, jsonStr);
    
    client.println("POST /publishData HTTP/1.1");
    client.println(host);
    client.println("Content-Type: application/json");
    client.print("Content-Length: ");
    client.println(jsonStr.length());
    client.println();
    client.println(jsonStr);
    
    while (client.available()) {
        String line = client.readStringUntil('\r');
        Serial.print(line);
    }
    
    client.stop();
}
