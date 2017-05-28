/*
 * Detect presence of Wifi devices and publish their mac/rssi to MQTT Server
 * Require Arduino Core for ESP8266 2.4.0-rc1 min
 *
 * Thanks to QuickFix for having opened the way
 * @see http://www.esp8266.com/viewtopic.php?t=14894&p=66200#p66200
 */

#include <Arduino.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>

#include "espresence.h"
#include "config.h"

ESP8266WiFiMulti wifiMulti;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
WiFiEventHandler probeRequestHandler;

bool otaInProgress = false;
clt_device_t devicelist[MAX_DEVICES];

void setup() {
    initSerial();
    initOTA();
    initWiFi();
    initMQTT();

    // Set probe request handler
    probeRequestHandler = WiFi.onSoftAPModeProbeRequestReceived(&onProbeRequest);
}

void loop() {

    // Handle wifi connection
    if (wifiMulti.run() != WL_CONNECTED) {
        WiFi.disconnect();
        wifiConnect();
        return;
    }

    // Handle OTA
    ArduinoOTA.handle();
    if (otaInProgress) {
        return;
    }

    // Handle MQTT
    if (!mqttClient.connected()) {
        mqttConnect();
    }
    mqttClient.loop();

    // Handle devices list
    checkList();
}

void checkList() {

    uint8_t i;
    for (i = 0; i < MAX_DEVICES; i++) {
        if (devicelist[i].mac != "" && (millis() - devicelist[i].ms > LIST_TIMEOUT * 1000)) {

            // Publish "Out" event
            char msg[90] = "";
            snprintf(msg, sizeof(msg),
                     "{\"event\":\"out\",\"mac\":\"%s\",\"rssi\":%d,\"uptime\":%d,\"heap\":%d}",
                     (devicelist[i].mac).c_str(), devicelist[i].rssi, getUptimeSecs(), ESP.getFreeHeap()
            );
            mqttClient.publish(MQTT_OUT_TOPIC, msg);

            // Clear expired device
            Serial.printf("[Device Out] MAC: %s\n", (devicelist[i].mac).c_str());
            devicelist[i] = {.mac = "", .rssi = 0, .ms = 0};
        }
    }

    // Check console and print list if \n char is detected
    char c;
    while (Serial.available() > 0) {
        c = Serial.read();
        if (c == '\n') {
            printlist();
        }
    }
}

void printlist() {

    uint8_t i, c = 0;

    // Count devices
    for (i = 0; i < MAX_DEVICES; i++) {
        if ((devicelist[i].mac != "") && (devicelist[i].rssi != 0)) {
            c++;
        }
    }
    Serial.printf("\n%d registered devices (prober running for %d seconds)\n", c, millis() / 1000UL);

    // List devices
    if (c > 0) {
        for (i = 0; i < MAX_DEVICES; i++) {
            if (devicelist[i].mac != "") {
                Serial.printf(
                        "MAC: %s, RSSI: %d, Last seen %d seconds ago\n",
                        (devicelist[i].mac).c_str(),
                        devicelist[i].rssi,
                        (millis() - devicelist[i].ms) / 1000UL
                );
            }
        }
    }
}

void onProbeRequest(const WiFiEventSoftAPModeProbeRequestReceived &evt) {

    String mac = macToString(evt.mac);
    //Serial.println(mac.c_str());

    uint8_t i;
    for (i = 0; i < MAX_DEVICES; i++) {
        if (devicelist[i].mac == mac) {

            // Update device
            if (evt.rssi > devicelist[i].rssi) {
                devicelist[i].rssi = evt.rssi; // Keep only max rssi
            }
            devicelist[i].ms = millis();

            return;
        }
    }

    for (i = 0; i < MAX_DEVICES; i++) {
        if (devicelist[i].mac == "") {

            //Add device
            devicelist[i] = {.mac = mac, .rssi = evt.rssi, .ms = millis()};
            Serial.printf("[Device In] MAC: %s, RSSI: %d\n", mac.c_str(), evt.rssi);

            // Publish "In" event
            char msg[90] = "";
            snprintf(msg, sizeof(msg),
                     "{\"event\":\"in\",\"mac\":\"%s\",\"rssi\":%d,\"uptime\":%d,\"heap\":%d}",
                     mac.c_str(), evt.rssi, getUptimeSecs(), ESP.getFreeHeap()
            );
            mqttClient.publish(MQTT_OUT_TOPIC, msg);

            return;
        }
    }

    Serial.println("Unable to add new device to list");
}

void initSerial() {
    Serial.begin(115200);
    Serial.setDebugOutput(SERIAL_SET_DEBUG_OUTPUT);
    Serial.println("\n\n*******************");
    Serial.printf("%s v%s\n\n", ESP_NAME, VERSION);
    Serial.printf("STA MAC:            %s\n", WiFi.macAddress().c_str());
    Serial.printf("AP MAC:             %s\n", WiFi.softAPmacAddress().c_str());
    Serial.printf("Chip ID:            %6X\n", ESP.getChipId());
    Serial.printf("Free space:         %s\n", prettyBytes(ESP.getFreeSketchSpace()).c_str());
    Serial.printf("Sketch size:        %s\n", prettyBytes(ESP.getSketchSize()).c_str());
    Serial.printf("Chip size:          %s\n", prettyBytes(ESP.getFlashChipRealSize()).c_str());
    Serial.printf("SDK version:        %s\n", ESP.getSdkVersion());
    Serial.println("*******************");
}

void initOTA() {

    ArduinoOTA.setHostname(ESP_NAME);

    ArduinoOTA.onStart([]() {
        otaInProgress = true;
        Serial.println("Start updating...");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.println(progress / (total / 100));
    });

    ArduinoOTA.onEnd([]() {
        Serial.println("\nend");
        otaInProgress = false;
    });

    ArduinoOTA.onError([](ota_error_t error) {
        String msg;
        if (error == OTA_AUTH_ERROR) msg = "auth failed";
        else if (error == OTA_BEGIN_ERROR) msg = "begin failed";
        else if (error == OTA_CONNECT_ERROR) msg = "connect failed";
        else if (error == OTA_RECEIVE_ERROR) msg = "receive failed";
        else if (error == OTA_END_ERROR) msg = "end failed";
        Serial.printf("Error: %s\n", msg.c_str());
    });

    ArduinoOTA.begin();
}

void wifiConnect() {
    while (wifiMulti.run() != WL_CONNECTED) {
        delay(500);
    }
    Serial.printf("Connected to %s with IP %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
}

void initWiFi() {

    WiFi.hostname(ESP_NAME);
    WiFi.mode(WIFI_AP_STA);

    // Connect station
    Serial.println("Connecting to WiFi...");
    uint8_t i, s = sizeof(AP_LIST) / sizeof AP_LIST[0];
    for (i = 0; i < s; i++) {
        wifiMulti.addAP(AP_LIST[i].ssid, AP_LIST[i].pwd);
    }
    wifiConnect();

    // Start AP
    WiFi.softAP(AP_SSID, AP_PASSWORD);
}

void mqttConnect() {
    while (!mqttClient.connected()) {
        Serial.println("Attempting MQTT connection...");
        if (mqttClient.connect(ESP_NAME)) {
            Serial.printf("Connected to %s\n", MQTT_HOST);
            mqttClient.publish(MQTT_OUT_TOPIC, "{\"event\":\"connected\"}");
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void initMQTT() {
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    mqttConnect();
}

uint32_t getUptimeSecs() {
    static uint32_t uptime = 0;
    static uint32_t previousMillis = 0;
    uint32_t now = millis();

    uptime += (now - previousMillis) / 1000UL;
    previousMillis = now;
    return uptime;
}

String prettyBytes(uint32_t bytes) {

    const char *suffixes[7] = {"B", "KB", "MB", "GB", "TB", "PB", "EB"};
    uint8_t s = 0;
    double count = bytes;

    while (count >= 1024 && s < 7) {
        s++;
        count /= 1024;
    }
    if (count - floor(count) == 0.0) {
        return String((int) count) + suffixes[s];
    } else {
        return String(round(count * 10.0) / 10.0, 1) + suffixes[s];
    };
}

String macToString(const uint8 mac[6]) {
    char buf[20];
    snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(buf);
}