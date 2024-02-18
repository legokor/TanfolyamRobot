#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include <ESP8266WebServer.h>

constexpr bool DEBUG_ENABLED = false;
constexpr int bufferSize = 128; 
ESP8266WebServer server(80);

enum class ReceiveState{
    CONFIG1,
    CONFIG2,
    DATA1,
    DATA2,
    IDLE
};

enum class EspState {
    PRECONFIG,
    WAITING_FOR_NETWORK,
    CONNECTING,
    CONNECTED
};

enum class ModuleStatus{
    CONNECTING_TO_WIFI,
    CONNECTING_TO_SERVER,
    CONNECTED_TO_SERVER,
    PRECONFIG
};

void setup() {
    Serial.setRxBufferSize(2048);
    Serial.begin(115200);
    dlog("");
    dlog("Serial started");

    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

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

    server.begin();
}

ModuleStatus moduleStatus = ModuleStatus::PRECONFIG;

void loop() {
    static ReceiveState receiveState = ReceiveState::IDLE;
    static char receiveOutBuffer[bufferSize];
    static int serialPtr = 0;
    static EspState espState = EspState::PRECONFIG;
    
    static char ssid[bufferSize / 2] = "";
    static char password[bufferSize / 2] = "";
    static char ipAddress[20] = "";

    bool configAvailable = false;
    bool dataAvailable = false;

    unsigned long now = millis();
    static unsigned long statusSendTime = 0;
    if(statusSendTime <= now){
        statusSendTime = now + 300;
        sendStatus(moduleStatus);
    }
    
    while(Serial.available()){
        static char serialBuffer[bufferSize];
        char c = Serial.read();

        switch(receiveState){
            case ReceiveState::IDLE:
                serialPtr = 0;
                if(c == 'C')
                    receiveState = ReceiveState::CONFIG1;
                if(c == 'D' && !configAvailable)
                    receiveState = ReceiveState::DATA1;
                break;
            case ReceiveState::CONFIG1:
                if(c == ':')
                    receiveState = ReceiveState::CONFIG2;
                else
                    receiveState = ReceiveState::IDLE;
                break;
            case ReceiveState::DATA1:
                if(c == ':')
                    receiveState = ReceiveState::DATA2;
                else
                    receiveState = ReceiveState::IDLE;
                break;
            case ReceiveState::CONFIG2:
                if(c == '\n'){
                    serialBuffer[serialPtr] = 0;
                    receiveState = ReceiveState::IDLE;
                    dataAvailable = false;
                    configAvailable = true;
                    strcpy(receiveOutBuffer, serialBuffer);
                }else{
                    serialBuffer[serialPtr++] = c;
                    if(serialPtr == bufferSize)
                        receiveState = ReceiveState::IDLE;
                }
                break;
            case ReceiveState::DATA2:
                if(c == '\n'){
                    serialBuffer[serialPtr] = 0;
                    receiveState = ReceiveState::IDLE;
                    dataAvailable = true;
                    strcpy(receiveOutBuffer, serialBuffer);
                }else{
                    serialBuffer[serialPtr++] = c;
                    if(serialPtr == bufferSize)
                        receiveState = ReceiveState::IDLE;
                }
                break;
        }
    }

    yield();

    if(configAvailable){
        char ssidTmp[bufferSize / 2];
        char passwordTmp[bufferSize / 2];
        if(sscanf(receiveOutBuffer, "\t%s\t%s\t%s", ssidTmp, passwordTmp, ipAddress) == 3){
            if(strcmp(ssidTmp, ssid) != 0 || strcmp(passwordTmp, password) != 0){
                strcpy(ssid, ssidTmp);
                strcpy(password, passwordTmp);
                if(espState == EspState::CONNECTED || espState == EspState::CONNECTING)
                    WiFi.disconnect();
                dlog("Config received");
                espState = EspState::WAITING_FOR_NETWORK;
            }
        }
    }

    switch(espState){
        case EspState::PRECONFIG:
            break;

        case EspState::WAITING_FOR_NETWORK:
            {
                moduleStatus = ModuleStatus::CONNECTING_TO_WIFI;
                int result = WiFi.scanComplete();
                if(result == -2){
                    dlog("Starting network scan...");
                    WiFi.scanNetworks(true, true);
                }else if(result >= 0){
                    bool networkFound = false;
                    for(int p = 0; p < result; p++){
                        if(strcmp(WiFi.SSID(p).c_str(), ssid) == 0){
                            networkFound = true;
                            dlog("Target network found");
                            dlog("Entering connecting state...");
                            WiFi.begin(ssid, password);
                            espState = EspState::CONNECTING;
                            break;
                        }
                    }
                    if(!networkFound){
                        dlog("Restarting network scan...");
                        WiFi.scanNetworks(true, true);
                    }
                }
            }
            break;

        case EspState::CONNECTING:
            if(WiFi.status() == WL_CONNECTED){
                dlog("Connected to WiFi network");
                espState = EspState::CONNECTED;
                moduleStatus = ModuleStatus::CONNECTING_TO_SERVER;
            }
            break;

        case EspState::CONNECTED:
            if(dataAvailable){
                dlog("Data received");
                processSerialData(receiveOutBuffer, ipAddress);
            }
            if(DEBUG_ENABLED){
                static unsigned long debugSendTime = 0;
                if(now >= debugSendTime){
                    debugSendTime = now + 250;
                    dlog("Sending dummy debug data...");
                    sendJson(ipAddress, 88, 69, 75, 1, 2, 3, 4, 22, 33, 44, 55, 1.2, 2.3, 3.4, 4.5, 5.6, 6.7, 7.8, "");
                }
            }
            break;
    }   

    server.handleClient();
    yield();
}

void dlog(const String& str){
    if(DEBUG_ENABLED){
        Serial.print("DEBUG: ");
        Serial.println(str);
    }
}

void sendStatus(ModuleStatus status){
    if(DEBUG_ENABLED)
        return;
    switch(status){
        case ModuleStatus::CONNECTING_TO_WIFI:
            Serial.print((char)1);
            break;
        case ModuleStatus::CONNECTING_TO_SERVER:
            Serial.print((char)2);
            break;
        case ModuleStatus::CONNECTED_TO_SERVER:
            Serial.print((char)3);
            break;
        case ModuleStatus::PRECONFIG:
            break;
    }
}

void processSerialData(const char *data, const char* ipaddr) {
    int servo, motora, motorb, cpsa, cpsb, cnta, cntb, usonic, hsv_h, hsv_s, hsv_v, rgb_r, rgb_g, rgb_b;
    char text[256] = "";
    
    sscanf(data, " %d %d %d %d %d %d %d %d %d %d %d %[^\"]", &rgb_r, &rgb_g, &rgb_b, &motora, &motorb, &cpsa, &cpsb, &cnta, &cntb, &servo, &usonic, text);
    rgb2hsv(rgb_r, rgb_g, rgb_b, &hsv_h, &hsv_s, &hsv_v);
    sendJson(ipaddr, servo, motora, motorb, cpsa, cpsb, cnta, cntb, usonic, hsv_h, hsv_s, hsv_v, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, text);
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

void sendJson(const char* ipaddr, int servo, int motora, int motorb, int cpsa, int cpsb, int cnta, int cntb, int usonic, int hsv_h, int hsv_s, int hsv_v, float imu_acc_a, float imu_acc_b, float imu_acc_c, float imu_gyro_x, float imu_gyro_y, float imu_gyro_z, float imu_temp, const char* text) {
    WiFiClient client;
    
    if (!client.connect(ipaddr, 5000)) {
        moduleStatus = ModuleStatus::CONNECTING_TO_SERVER;
        dlog("Connection failed");
        return;
    }
    moduleStatus = ModuleStatus::CONNECTED_TO_SERVER;

    StaticJsonDocument<512> doc;
    
    doc["servo"] = servo;
    doc["motora"] = motora;
    doc["motorb"] = motorb;
    doc["cpsa"] = cpsa;
    doc["cpsb"] = cpsb;
    doc["cnta"] = cnta;
    doc["cntb"] = cntb;
    doc["usonic"] = usonic;
    if(strlen(text) > 0)
        doc["consolemessage"] = String(text);
    
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
    
    char host[30] = "Host: ";
    strcat(host, ipaddr);
    client.println("POST /publishData HTTP/1.1");
    client.println(host);
    client.println("Content-Type: application/json");
    client.print("Content-Length: ");
    client.println(jsonStr.length());
    client.println();
    client.println(jsonStr);
    
    // while (client.available()) {
    //     String line = client.readStringUntil('\r');
    //     Serial.print(line);
    // }
    
    client.stop();
}
