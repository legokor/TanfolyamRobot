#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>

const char* ssid = "Andi";
const char* password = "Gomboc02";
const char* ipaddr = "192.168.0.100";
char[30] host = "Host: ";


const int bufferSize = 128;  // Adjust the buffer size as needed
char serialBuffer[bufferSize] = {};
char htmlMessage[bufferSize] = {};
int bufferIndex = 0;

enum State {
  PRECONFIG,
  IDLE,
  READING
};

State currentState = PRECONFIG;

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println("Serial started");


    while(currentState != IDLE){        
      if (Serial.available() > 0) {
        char incomingChar = Serial.read();
    
        switch (currentState) {
          case PRECONFIG:
            if (incomingChar == 'C') {
              currentState = READING;
              bufferIndex = 0;
            }
            break;
    
          case READING:
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

    sscanf(serialBuffer, ":\t%s\t%s\t%s", ssid, password, ipaddr);
    strcat(host, ipaddr);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }

    Serial.println("Connected to WiFi");
}

void loop() {
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

    
    // TODO: is this necessary? Isn't the output of % always +?

}

void processSerialData(const char *data) {
  //Serial.print("Received Data: ");
  //Serial.println(data);
  //strcpy(htmlMessage, data);
  int servo, motora, motorb, usonic, hsv_h, hsv_s, hsv_v, rgb_r, rgb_g, rgb_b;
  float wheelaspeed, wheelbspeed, imu_acc_a, imu_acc_b, imu_acc_c, imu_gyro_x, imu_gyro_y, imu_gyro_z, imu_temp;
  sscanf(data, ": %d %d %d %f %f %ld %ld %d %d", &rgb_r, &rgb_g, &rgb_b, &wheelaspeed, &wheelbspeed, &motora, &motorb, &servo, &usonic);
    rgb2hsv(rgb_r, rgb_g, rgb_b, &hsv_h, &hsv_s, &hsv_v);
    sendJson(servo, motora, motorb, wheelaspeed, wheelbspeed, usonic, hsv_h, hsv_s, hsv_v, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) ;

}


void sendJson(int servo, int motora, int motorb, float wheelaspeed, float wheelbspeed, int usonic, int hsv_h, int hsv_s, int hsv_v, float imu_acc_a, float imu_acc_b, float imu_acc_c, float imu_gyro_x, float imu_gyro_y, float imu_gyro_z, float imu_temp) {
    WiFiClient client;
    
    if (!client.connect(ipaddr, 5000)) {
        Serial.println("Connection failed");
        return;
    }
    Serial.println("NA CSAAAAA");
    
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
