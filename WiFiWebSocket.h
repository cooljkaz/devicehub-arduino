#ifndef WIFI_WEB_SOCKET_H
#define WIFI_WEB_SOCKET_H

#include <WiFi.h>
#include <WebSocketsServer.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include "OtaHelper.h"

class WiFiWebSocket {
public:
    WiFiWebSocket(const char* ssid, const char* password, const char* deviceName, int port, int udpPort, const char* staticIPStr = nullptr);
    void begin();
    void loop();
    void sendStatus(const char* status);
    void sendMessage(const char* message);

private:
    const char* ssid;
    const char* password;
    const char* deviceName;
    int port;
    int udpPort;
    IPAddress staticIP;
    IPAddress gateway;
    IPAddress subnet;
    bool useStaticIP;
    WebSocketsServer webSocketServer;
    WiFiUDP udp;
    OtaHelper otaHelper;

    void connectWiFi();
    void handleUDP();
    String getDeviceInfo();
    static void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
    void configureStaticIP(const char* staticIPStr);
};

#endif
