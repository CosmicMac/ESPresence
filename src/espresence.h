#ifndef ESPRESENCE_ESPRESENCE_H
#define ESPRESENCE_ESPRESENCE_H

typedef struct {
    const char *ssid;
    const char *pwd;
} ap_t;

typedef struct {
    String mac;
    int32_t rssi;
    uint32_t ms;
} clt_device_t;

void checkList();
void printlist();
void onProbeRequest(const WiFiEventSoftAPModeProbeRequestReceived &evt);
void wifiConnect();
void initSerial();
void initOTA();
void initWiFi();
void initMQTT();
void mqttConnect();
uint32_t getUptimeSecs();
String prettyBytes(uint32_t bytes);
String macToString(const uint8 mac[6]);

#endif //ESPRESENCE_ESPRESENCE_H
