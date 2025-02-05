#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>

constexpr bool DEBUG_ENABLED = false;
constexpr int bufferSize = 250; 
ESP8266WebServer server(80);

const String COLOR = "red";      //might be red, green, gray, pink, black (according to the server/UI code)

enum class ReceiveState{
    CONFIG,
    DATA,
    TEXT,
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

struct __attribute__((__packed__)) tel_OutputData
{
	uint8_t r, g, b;
	int8_t setPointRightM, setPointLeftM;
	int16_t cpsRightEnc, cpsLeftEnc;
	int32_t cntRightEnc, cntLeftEnc;
	int8_t servoPos;
	uint8_t distance;
	int16_t accX, accY, accZ;
	int16_t gyroX, gyroY, gyroZ;
	int16_t magX, magY, magZ;
	int16_t temp;
	int16_t pitch, roll;
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
    static char receiveOutBuffer[bufferSize];
    static char textBuffer[bufferSize*4];
    static int serialPtr = 0;
    static EspState espState = EspState::PRECONFIG;
    
    static char ssid[bufferSize / 2] = "";
    static char password[bufferSize / 2] = "";
    static char ipAddress[20] = "";

    static tel_OutputData data;

    bool configAvailable = false;
    bool dataAvailable = false;
    bool textAvailable = false;

    unsigned long now = millis();
    static unsigned long statusSendTime = 0;
    if(statusSendTime <= now){
        statusSendTime = now + 300;
        sendStatus(moduleStatus);
    }

    ReceiveState receiveState = processSerialData(receiveOutBuffer, bufferSize);

    switch (receiveState)
    {
    case ReceiveState::CONFIG:
        configAvailable = true;
        break;
    case ReceiveState::DATA:
        memcpy(&data, receiveOutBuffer, sizeof(tel_OutputData));
        dataAvailable = true;
        break;
    case ReceiveState::TEXT:
        if (textBuffer[0] == 0)
            strcpy(textBuffer, receiveOutBuffer);
        else
        {
            size_t len = strlen(textBuffer);
            if (4 * bufferSize - len - 2 < strlen(receiveOutBuffer))
                break;
            textBuffer[len] = '\n';
            strcpy(textBuffer + len + 1, receiveOutBuffer);
        }
        break;
    }

    yield();

    if(configAvailable){
        char ssidTmp[bufferSize / 2];
        char passwordTmp[bufferSize / 2];
        if((!DEBUG_ENABLED && sscanf(receiveOutBuffer, "\t%s\t%s\t%s", ssidTmp, passwordTmp, ipAddress) == 3) 
            || (DEBUG_ENABLED && sscanf(receiveOutBuffer, "%s %s %s", ssidTmp, passwordTmp, ipAddress) == 3)){
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
                processData(&data, textBuffer, ipAddress);
            }
            if(DEBUG_ENABLED){
                static unsigned long debugSendTime = 0;
                if(now >= debugSendTime){
                    debugSendTime = now + 250;
                    dlog("Sending dummy debug data...");
                    sendJson(ipAddress, 88, 69, 75, 1, 2, 3, 4, 22, 33, 44, 55, 1.2, 2.3, 3.4, 4.5, 5.6, 6.7, 7.8, 1, 1, 1, 33, 44, "");
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

ReceiveState processSerialData(char* buffer, size_t maxSize)
{
    static int serialPtr = 0;
    static ReceiveState receiveState = ReceiveState::IDLE;
    static bool start = true;
    static bool escapeActive = false;

    while (Serial.available())
    {
        char c = Serial.read();
        if (c == '\n')
        {
            buffer[serialPtr] = 0;
            ReceiveState tmp = receiveState;
            receiveState = ReceiveState::IDLE;
            start = true;
            escapeActive = false;
            if (serialPtr == 0 || (tmp == ReceiveState::DATA && serialPtr != sizeof(tel_OutputData)))
            {
                serialPtr = 0;
                return ReceiveState::IDLE;
            }
            serialPtr = 0;
            return tmp;
        }
        else if (start)
        {
            if (c == 'C')
            {
                receiveState = ReceiveState::CONFIG;
                start = false;
            }
            else if (c == 'D')
            {
                receiveState = ReceiveState::DATA;
                start = false;
            }
            else if (c == 'T')
            {
                receiveState = ReceiveState::TEXT;
                start = false;
            }
        }
        else if (escapeActive)
        {
            escapeActive = false;
            if (c == '0')
                buffer[serialPtr++] = '\n';
            else if (c == '1')
                buffer[serialPtr++] = '~';
            else
            {
                serialPtr = 0;
                start = true;
            }
        }
        else if (c == '~')
        {
            escapeActive = true;
        }
        else
        {
            buffer[serialPtr++] = c;
        }

        if (serialPtr >= maxSize)
        {
            receiveState = ReceiveState::IDLE;
            start = true;
            escapeActive = false;
            serialPtr = 0;
        }
    }

    return ReceiveState::IDLE;
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

void processData(tel_OutputData* data, char* text, const char* ipaddr) {
    int hsv_h, hsv_s, hsv_v;
    rgb2hsv(data->r, data->g, data->b, &hsv_h, &hsv_s, &hsv_v);
    if(sendJson(ipaddr, data->servoPos, data->setPointRightM, data->setPointLeftM, data->cpsRightEnc, data->cpsLeftEnc, data->cntRightEnc, data->cntLeftEnc, data->distance, hsv_h, hsv_s, hsv_v, 
                data->accX / 100.0f, data->accY / 100.0f, data->accZ / 100.0f, data->gyroX, data->gyroY, data->gyroZ, data->magX / 10.0f, data->magY / 10.0f, data->magZ / 10.0f, data->temp / 10.0f, data->pitch / 10.0f, data->roll / 10.0f, text))
        text[0] = '\0';
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

bool sendJson(const char* ipaddr, int servo, int motora, int motorb, int cpsa, int cpsb, int cnta, int cntb, int usonic, int hsv_h, int hsv_s, int hsv_v, 
                float imu_acc_a, float imu_acc_b, float imu_acc_c, float imu_gyro_x, float imu_gyro_y, float imu_gyro_z, float imu_mag_x, float imu_mag_y, 
                float imu_mag_z, float imu_temp, float pitch, float roll, const char* text) {
    WiFiClient client;
    HTTPClient http;
    http.setTimeout(300);
    http.begin(client, ipaddr, 5000, "/publishData");
    http.addHeader("Content-Type", "application/json");

    static StaticJsonDocument<512> doc;
    doc.clear();
    
    doc["color"] = COLOR;
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
    imu_acc["x"] = imu_acc_a;
    imu_acc["y"] = imu_acc_b;
    imu_acc["z"] = imu_acc_c;
    
    JsonObject imu_gyro = imu.createNestedObject("gyro");
    imu_gyro["x"] = imu_gyro_x;
    imu_gyro["y"] = imu_gyro_y;
    imu_gyro["z"] = imu_gyro_z;

    JsonObject imu_mag = imu.createNestedObject("mag");
    imu_mag["x"] = imu_mag_x;
    imu_mag["y"] = imu_mag_y;
    imu_mag["z"] = imu_mag_z;

    JsonObject imu_angle = imu.createNestedObject("angle");
    imu_angle["pitch"] = pitch;
    imu_angle["roll"] = roll;
    
    imu["temp"] = imu_temp;
    
    String jsonStr;
    serializeJson(doc, jsonStr);
    
    int code = http.POST(jsonStr);
    if(code < 0) {
        moduleStatus = ModuleStatus::CONNECTING_TO_SERVER;
        dlog("Connection failed");
        http.end();
        return false;
    }
    dlog(String("Server responded with code: ") + code);
    moduleStatus = ModuleStatus::CONNECTED_TO_SERVER;
    
    // while (client.available()) {
    //     String line = client.readStringUntil('\r');
    //     Serial.print(line);
    // }
    
    http.end();
    return true;
}
