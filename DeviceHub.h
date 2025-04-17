#ifndef DEVICEHUB_H
#define DEVICEHUB_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <map>
#include <vector>
#include "OtaHelper.h"
#include <coap-simple.h>
#include <functional>

// Protocol versions
#define DEVICEHUB_VERSION_2 "2.0.0"

// CoAP ports
#define COAP_PORT 5683
#define SECURE_COAP_PORT 5684
#define DISCOVERY_PORT 8888
static const uint16_t EMERGENCY_PORT = 8890;
static const uint16_t EMERGENCY_NOTIFICATION_PORT = 8891;

enum class DeviceState {
    Normal,
    Emergency,
    Idle,
    Active,
    Error
};

struct ActionParameter {
    String name;
    String type;
    String unit;
    float defaultValue;
    float min;
    float max;
};

class DeviceHub {
public:
    using ActionCallback = std::function<String(const JsonObject&)>;
    using EventCallback = std::function<void(const JsonObject&)>;

    struct Action {
        String id;
        String name;
        std::vector<ActionParameter> parameters;
        ActionCallback callback;
    };

    struct Event {
        String id;
        String name;
        JsonObject dataSchema;
    };

    struct Setting {
        String id;
        String name;
        String type;
        JsonVariant defaultValue;
        float min;
        float max;
    };

    struct Observer {
        IPAddress ip;
        int port;
        String version;
        unsigned long lastUpdate;
    };

    struct PendingMessage {
        String payload;
        unsigned long timestamp;
        int retries = 0;
    };

    DeviceHub(const char* ssid, const char* password, const char* deviceName, const char* deviceType = "esp32");
    ~DeviceHub();
    void begin();
    void begin(HardwareSerial& serial);
    void loop();
    
    // Action registration
    void registerAction(const String& actionId, ActionCallback callback);
    void registerAction(const String& actionId, const String& actionName, ActionCallback callback);
    void registerAction(const String& actionId, const String& actionName, ActionCallback callback, const std::vector<ActionParameter>& parameters);
    
    // Event registration and emission
    void registerEvent(const String& eventId, const String& eventName, const JsonObject& schema = JsonObject());
    void emitEvent(const String& eventId, const JsonObject& payload = JsonObject());
    
    // Persistent storage
    void savePersistentData(const char* key, const String& value);
    String loadPersistentData(const char* key, const String& defaultValue = "");
    
    // Device identification
    void setDeviceUUID(const String& uuid);
    String getDeviceUUID();
    
    // Emergency handling
    void registerEmergencyAction(ActionCallback callback);
    void registerResetAction(ActionCallback callback);
    
    // Version management
    static const char* API_VERSION;
    String getSupportedVersions();
    bool isVersionSupported(const String& version);
    String getDeviceInfo(const String& version);  // Version-specific device info
    String getDeviceInfo();                         // Convenience overload without parameters
    
    // Message sending (legacy UDP)
    void sendMessage(const String& message, const String& type = "message");
    void sendEmergency(const String& message);
    void sendEmergencyResponse(const char* message);

    // Debug methods
    void enableDebug(HardwareSerial& serial);
    void disableDebug();
    void log(const char* format, ...);

    // Static members
    static const IPAddress BROADCAST_IP;

    // CoAP Helper
    String processRequest(const JsonObject& doc);
    void sendCoAPResponse(IPAddress ip, int port, uint16_t messageId, 
                          const String& payload, 
                          COAP_RESPONSE_CODE code,
                          COAP_CONTENT_TYPE type,
                          const uint8_t *token, 
                          int tokenlen);

private:
    const char* ssid;
    const char* password;
    const char* deviceName;
    const char* deviceType;
    String deviceUUID;
    DeviceState currentState;
    bool dtlsEnabled;
    HardwareSerial* debugSerial;
    bool debugEnabled;

    WiFiUDP udp;
    WiFiUDP discoveryUdp;
    WiFiUDP emergencyUdp;

    Coap *coap;
    Coap *secureCoap;
    

    std::map<String, Action> actions;
    std::map<String, Event> events;
    std::map<uint16_t, PendingMessage> pendingAcks;
    std::vector<Observer> observers;

    OtaHelper otaHelper;

    ActionCallback emergencyAction = nullptr;
    ActionCallback resetAction = nullptr;

    void ensureDeviceUUID();
    void generateAndStoreUUID();
    bool isUUIDValid();
    void setupCoAPResources();
    void handleCoAPRequest(CoapPacket& packet, IPAddress ip, int port);
    String handleAction(const String& actionId, const JsonObject& payload);
    void notifyObservers(const String& eventId, const JsonObject& payload, const String& version);
    void sendCoAPMessage(const String& payload);
    void sendCoAPMessage(const String& payload, COAP_TYPE messageType);
    void resendPendingMessages();
    void sendPeriodicUpdate();
    void returnToNormalState();
    void sendAck(const String& messageId);
    void sendUdpMessage(const String& payload);

    void connectWiFi();

    void handleIncomingPacket();
    void sendDeviceInfo();

    void handleEmergencyPacket();
    void handleEmergencyStart();
    void handleEmergencyEnd();
};

#endif // DEVICEHUB_H
