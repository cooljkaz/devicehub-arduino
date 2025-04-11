#ifndef DEVICEHUB_H
#define DEVICEHUB_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <map>
#include <vector>
#include "OtaHelper.h"

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

    DeviceHub(const char* ssid, const char* password, const char* deviceName, const char* deviceType = "generic");
    void begin();
    void loop();
    
    // Action registration
    void registerAction(const String& actionId, ActionCallback callback);
    void registerAction(const String& actionId, const String& actionName, ActionCallback callback, const std::vector<ActionParameter>& parameters = {});
    
    // Required emergency actions
    void registerEmergencyAction(ActionCallback callback);
    void registerResetAction(ActionCallback callback);
    
    // Event registration
    void registerEvent(const String& eventId, const String& eventName, const JsonObject& dataSchema = {});
    void emitEvent(const String& eventId, const JsonObject& payload = {});
    
    // Message sending
    void sendMessage(const String& message, const String& type = "device_message");
    void sendEmergency(const String& message);

    // Persistent data methods
    void savePersistentData(const char* key, const String& value);
    String loadPersistentData(const char* key, const String& defaultValue = "");

    // Device UUID methods
    void ensureDeviceUUID();
    void generateAndStoreUUID();
    bool isUUIDValid();
    void setDeviceUUID(const String& uuid);

    // Device UUID access
    String getDeviceUUID();

private:
    static const uint16_t LOCAL_PORT = 8888;
    static const uint16_t HUB_PORT = 8889;
    static const uint16_t EMERGENCY_PORT = 8890;
    static const uint16_t EMERGENCY_NOTIFICATION_PORT = 8891;
    static const IPAddress BROADCAST_IP;
    static const char* API_VERSION;

    const char* ssid;
    const char* password;
    const char* deviceName;
    const char* deviceType;
    String deviceUUID;
    
    WiFiUDP udp;
    WiFiUDP emergencyUdp;
    
    OtaHelper otaHelper;

    std::map<String, ActionCallback> actionsCallbacks;
    std::vector<String> actionNames;
    // Map to store allowed fields for each registered action
    std::map<String, std::vector<String>> allowedFieldsMap;
    
    // Action and event storage
    std::map<String, Action> actions;
    std::map<String, Event> events;
    
    ActionCallback emergencyAction;
    ActionCallback resetAction;

    DeviceState currentState;

    struct PendingMessage {
        String payload;
        unsigned long timestamp;
        int retries;
    };
    std::map<String, PendingMessage> pendingAcks;

    void connectWiFi();
    void handleIncomingPacket();
    void handleEmergencyPacket();
    void handleEmergencyStart();
    void handleEmergencyEnd();
    void performEmergencyActions();
    void returnToNormalState();
    void sendEmergencyResponse(const char* message);
    void sendDeviceInfo();
    String getDeviceInfo();
    void resendPendingMessages();
    String handleAction(const String& actionId, const JsonObject& payload);
    void sendAck(const String& messageId);
    void sendPeriodicUpdate();
    void sendUdpMessage(const String& payload);
    
};

#endif // DEVICEHUB_H
