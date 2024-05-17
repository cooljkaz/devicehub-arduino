#include "WiFiWebSocket.h"

// Define the global reference
WiFiWebSocket* globalWiFiWebSocket = nullptr;

WiFiWebSocket::WiFiWebSocket(const char* ssid, const char* password, const char* deviceName, int port, int udpPort, const char* staticIPStr)
    : ssid(ssid), password(password), deviceName(deviceName), port(port), udpPort(udpPort), webSocketServer(port), otaHelper(), useStaticIP(staticIPStr != nullptr) {
    // Set the global reference to this instance
    globalWiFiWebSocket = this;
    if (useStaticIP) {
        configureStaticIP(staticIPStr);
    }
}

void WiFiWebSocket::configureStaticIP(const char* staticIPStr) {
    staticIP.fromString(staticIPStr);
    gateway = IPAddress(staticIP[0], staticIP[1], staticIP[2], 1); // Assuming gateway is the first IP in the subnet
    subnet = IPAddress(255, 255, 255, 0); // Default subnet
}

void WiFiWebSocket::connectWiFi() {
    if (useStaticIP) {
        if (!WiFi.config(staticIP, gateway, subnet)) {
            Serial.println("STA Failed to configure");
        }
    }
    WiFi.begin(ssid, password);
    Serial.println("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(1000);
    }
    Serial.println("\nConnected to WiFi");
}

void WiFiWebSocket::begin() {
    Serial.println("Starting WiFiWebSocket...");
    connectWiFi();
    // Start WebSocket server
    webSocketServer.begin();
    webSocketServer.onEvent(webSocketEvent);

    // Start UDP
    udp.begin(udpPort);

    // Initialize OTA
    otaHelper.start(ssid, password, deviceName, password, 3232, 115200);
}

void WiFiWebSocket::loop() {
    // Check WiFi connection status and reconnect if necessary
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi connection lost. Reconnecting...");
        connectWiFi();
    } else {
        webSocketServer.loop();
        handleUDP();
    }

    // Handle OTA updates
    otaHelper.handle();
}

void WiFiWebSocket::sendStatus(const char* status) {
    webSocketServer.broadcastTXT(status);
}

void WiFiWebSocket::sendMessage(const char* message) {
    webSocketServer.broadcastTXT(message);
}

void WiFiWebSocket::handleUDP() {
    int packetSize = udp.parsePacket();
    if (packetSize) {
        char incomingPacket[255];
        int len = udp.read(incomingPacket, 255);
        if (len > 0) {
            incomingPacket[len] = 0;
        }
        Serial.printf("UDP packet received from %s:%d - %s\n", udp.remoteIP().toString().c_str(), udp.remotePort(), incomingPacket);

        StaticJsonDocument<200> doc;
        DeserializationError error = deserializeJson(doc, incomingPacket);
        if (error) {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.f_str());
            return;
        }

        if (doc["type"] == "REQUEST_DEVICE_INFO") {
            String deviceInfo = getDeviceInfo();
            udp.beginPacket(udp.remoteIP(), 8889); // Respond to the listener port
            udp.write(reinterpret_cast<const uint8_t*>(deviceInfo.c_str()), deviceInfo.length());
            udp.endPacket();
            Serial.printf("Sent device info to %s:%d - %s\n", udp.remoteIP().toString().c_str(), 8889, deviceInfo.c_str());
        }
    }
}

String WiFiWebSocket::getDeviceInfo() {
    StaticJsonDocument<200> doc;
    doc["name"] = deviceName;
    doc["ip"] = WiFi.localIP().toString();
    doc["port"] = port;

    String jsonString;
    serializeJson(doc, jsonString);
    return jsonString;
}

void WiFiWebSocket::webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
    if (globalWiFiWebSocket) {
        switch (type) {
            case WStype_DISCONNECTED:
                Serial.printf("[%u] Disconnected!\n", num);
                globalWiFiWebSocket->webSocketServer.begin();
                Serial.println("WebSocket server restarted after disconnection.");
                break;
            case WStype_CONNECTED: {
                IPAddress ip = globalWiFiWebSocket->webSocketServer.remoteIP(num);
                Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
                globalWiFiWebSocket->webSocketServer.sendTXT(num, "Connected");
            }
                break;
            case WStype_TEXT:
                Serial.printf("[%u] Text: %s\n", num, payload);
                break;
            case WStype_PONG:
                Serial.printf("[%u] Pong received\n", num);
                break;
            default:
                break;
        }
    }
}
