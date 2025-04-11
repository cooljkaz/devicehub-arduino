#include "DeviceHub.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <Preferences.h>

Preferences prefs;
const IPAddress DeviceHub::BROADCAST_IP(255, 255, 255, 255);
const char* DeviceHub::API_VERSION = "1.0.0";

DeviceHub::DeviceHub(const char* ssid, const char* password, const char* deviceName, const char* deviceType)
    : ssid(ssid), password(password), deviceName(deviceName), deviceType(deviceType), 
      currentState(DeviceState::Normal) {}

void DeviceHub::begin() {
    Serial.begin(115200);
    connectWiFi();
    
    if (udp.begin(LOCAL_PORT)) {
        Serial.printf("UDP socket bound to port %d\n", LOCAL_PORT);
    } else {
        Serial.println("Failed to bind UDP socket");
    }

    if (emergencyUdp.begin(EMERGENCY_PORT)) {
        Serial.printf("Emergency UDP socket bound to port %d\n", EMERGENCY_PORT);
    } else {
        Serial.println("Failed to bind emergency UDP socket");
    }
    
    ensureDeviceUUID();
    otaHelper.start(ssid, password, deviceName, password, 3232, 115200);
}

void DeviceHub::loop() {
    if (WiFi.status() != WL_CONNECTED) {
        connectWiFi();
    }
    
    handleIncomingPacket();
    handleEmergencyPacket();
    resendPendingMessages();
    sendPeriodicUpdate();
    otaHelper.handle();

}

void DeviceHub::connectWiFi() {
    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi");
}

void DeviceHub::handleIncomingPacket() {
    int packetSize = udp.parsePacket();
    if (packetSize) {
        char incomingPacket[255];
        int len = udp.read(incomingPacket, 255);
        if (len > 0) {
            incomingPacket[len] = 0;
        }
        Serial.printf("UDP packet received from %s:%d - %s\n", udp.remoteIP().toString().c_str(), udp.remotePort(), incomingPacket);

        StaticJsonDocument<512> doc;
        DeserializationError error = deserializeJson(doc, incomingPacket);
        if (error) {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.f_str());
            return;
        }

        if (doc.containsKey("type")) {
            String messageType = doc["type"].as<String>();

            if (messageType == "REQUEST_DEVICE_INFO") {
                Serial.println("Received REQUEST_DEVICE_INFO");
                sendDeviceInfo();
            }
            else if (messageType == "action_request") {
                if (doc.containsKey("id") && doc.containsKey("action")) {
                    String messageId = doc["id"].as<String>();
                    String actionId = doc["action"].as<String>();
                    Serial.printf("Received action request with ID: %s, Action: %s\n", messageId.c_str(), actionId.c_str());
                    
                    JsonObject payload = doc["payload"].as<JsonObject>();
                    String result = handleAction(actionId, payload);
                }
                else {
                    Serial.println("Received malformed action request");
                }
            }
            else {
                Serial.printf("Received unknown message type: %s\n", messageType.c_str());
            }
        }
        else {
            Serial.println("Received message without type field");
        }
    }
}

void DeviceHub::handleEmergencyPacket() {
    int packetSize = emergencyUdp.parsePacket();
    if (packetSize) {
        char incomingPacket[255];
        int len = emergencyUdp.read(incomingPacket, 255);
        if (len > 0) {
            incomingPacket[len] = 0;
        }
        Serial.printf("Emergency UDP packet received from %s:%d - %s\n", emergencyUdp.remoteIP().toString().c_str(), emergencyUdp.remotePort(), incomingPacket);

        StaticJsonDocument<512> doc;
        DeserializationError error = deserializeJson(doc, incomingPacket);
        if (error) {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.f_str());
            return;
        }

        if (doc["type"] == "emergency_action") {
            handleEmergencyStart();
        } else if (doc["type"] == "emergency_end") {
            handleEmergencyEnd();
        }
    }
}

void DeviceHub::handleEmergencyStart() {
    if (currentState != DeviceState::Emergency) {
        currentState = DeviceState::Emergency;
        Serial.println("Entering emergency state");
        performEmergencyActions();
        sendEmergencyResponse("Emergency state entered");
    }
}

void DeviceHub::handleEmergencyEnd() {
    if (currentState == DeviceState::Emergency) {
        currentState = DeviceState::Normal;
        Serial.println("Exiting emergency state");
        returnToNormalState();
        sendEmergencyResponse("Returned to normal state");
    }
}

void DeviceHub::performEmergencyActions() {
    Serial.println("Attempting emergency protocol...");
    if (emergencyAction) {
        String result = emergencyAction(JsonObject());
        sendMessage("Emergency action performed: " + result);
    }
}

void DeviceHub::returnToNormalState() {
    if (resetAction) {
        String result = resetAction(JsonObject());
        sendMessage("Reset action performed: " + result);
    }
    sendMessage("Returned to normal state");
}

void DeviceHub::sendEmergencyResponse(const char* message) {
    StaticJsonDocument<256> doc;
    doc["type"] = "emergency_action_response";
    doc["version"] = API_VERSION;
    doc["deviceName"] = deviceName;
    doc["message"] = message;

    String responseJson;
    serializeJson(doc, responseJson);

    emergencyUdp.beginPacket(emergencyUdp.remoteIP(), emergencyUdp.remotePort());
    emergencyUdp.print(responseJson);
    emergencyUdp.endPacket();

    Serial.printf("Sent emergency response: %s\n", responseJson.c_str());
}

String DeviceHub::getDeviceInfo() {
    StaticJsonDocument<2048> doc;
    doc["type"] = "device_info";
    doc["version"] = API_VERSION;
    doc["uuid"] = deviceUUID;
    doc["name"] = deviceName;
    doc["deviceType"] = deviceType;

    // Add capabilities
    JsonObject capabilities = doc.createNestedObject("capabilities");

    // Add actions
    JsonArray actionsArray = capabilities.createNestedArray("actions");
    for (const auto& actionPair : actions) {
        const Action& action = actionPair.second;
        JsonObject actionObj = actionsArray.createNestedObject();
        actionObj["id"] = action.id;
        actionObj["name"] = action.name;
        
        JsonArray parametersArray = actionObj.createNestedArray("parameters");
        for (const auto& param : action.parameters) {
            JsonObject paramObj = parametersArray.createNestedObject();
            paramObj["name"] = param.name;
            paramObj["type"] = param.type;
            paramObj["unit"] = param.unit;
            paramObj["defaultValue"] = param.defaultValue;
            paramObj["min"] = param.min;
            paramObj["max"] = param.max;
        }
    }

    // Add events
    JsonArray eventsArray = capabilities.createNestedArray("events");
    for (const auto& eventPair : events) {
        const Event& event = eventPair.second;
        JsonObject eventObj = eventsArray.createNestedObject();
        eventObj["id"] = event.id;
        eventObj["name"] = event.name;
        eventObj["dataSchema"] = event.dataSchema;
    }

    // Add status
    JsonObject status = doc.createNestedObject("status");
    switch (currentState) {
        case DeviceState::Normal:
            status["state"] = "normal";
            break;
        case DeviceState::Emergency:
            status["state"] = "emergency";
            break;
        case DeviceState::Idle:
            status["state"] = "idle";
            break;
        case DeviceState::Active:
            status["state"] = "active";
            break;
        case DeviceState::Error:
            status["state"] = "error";
            break;
    }

    String jsonString;
    serializeJson(doc, jsonString);
    return jsonString;
}

String DeviceHub::getDeviceUUID() {
    if(!deviceUUID) {
        ensureDeviceUUID();
    }
    return deviceUUID;
}

void DeviceHub::sendMessage(const String& message, const String& type) {
    StaticJsonDocument<256> doc;
    doc["type"] = type;
    doc["version"] = API_VERSION;
    doc["deviceName"] = deviceName;
    doc["message"] = message;
    doc["uuid"] = deviceUUID;
    

    String jsonString;
    serializeJson(doc, jsonString);
    
    Serial.printf("Sending message of type %s: %s\n", type.c_str(), message.c_str());
    sendUdpMessage(jsonString);
}

void DeviceHub::sendEmergency(const String& message) {
    StaticJsonDocument<256> doc;
    doc["type"] = "emergency";
    doc["version"] = API_VERSION;
    doc["deviceName"] = deviceName;
    doc["message"] = message;

    String jsonString;
    serializeJson(doc, jsonString);
    
    udp.beginPacket(BROADCAST_IP, EMERGENCY_NOTIFICATION_PORT);
    udp.print(jsonString);
    udp.endPacket();
}

void DeviceHub::sendUdpMessage(const String& payload) {
    udp.beginPacket(BROADCAST_IP, HUB_PORT);
    udp.print(payload);
    udp.endPacket();
}

void DeviceHub::resendPendingMessages() {
    unsigned long currentTime = millis();
    for (auto it = pendingAcks.begin(); it != pendingAcks.end(); ) {
        if (currentTime - it->second.timestamp > 5000) {  // 5 seconds timeout
            if (it->second.retries < 3) {  // Maximum 3 retries
                sendMessage(it->second.payload);
                it->second.timestamp = currentTime;
                it->second.retries++;
                ++it;
            } else {
                it = pendingAcks.erase(it);
            }
        } else {
            ++it;
        }
    }
}

void DeviceHub::registerAction(const String& actionId, ActionCallback callback) {
    Action action;
    action.id = actionId;
    action.name = actionId;
    action.callback = callback;
    actions[actionId] = action;
}

void DeviceHub::registerAction(const String& actionId, const String& actionName, ActionCallback callback, const std::vector<ActionParameter>& parameters) {
    Action action;
    action.id = actionId;
    action.name = actionName;
    action.parameters = parameters;
    action.callback = callback;
    actions[actionId] = action;
}

String DeviceHub::handleAction(const String& actionId, const JsonObject& payload) {
    auto it = actions.find(actionId);
    if (it != actions.end()) {
        String responseStatus = "success";
        String responseMessage = "";
        
        try {
            responseMessage = it->second.callback(payload);
        } catch (const std::exception& e) {
            Serial.printf("Error handling action %s: %s\n", actionId.c_str(), e.what());
            responseStatus = "error";
            responseMessage = String(e.what());
        }
        
        // Prepare action response
        StaticJsonDocument<512> responseDoc;
        responseDoc["type"] = "action_response";
        responseDoc["id"] = payload["id"] | "unknown";
        responseDoc["status"] = responseStatus;
        responseDoc["message"] = responseMessage;
        responseDoc["timestamp"] = millis() / 1000;

        // Send UDP response to hub's action response port
        String responseJson;
        serializeJson(responseDoc, responseJson);
        
        udp.beginPacket(BROADCAST_IP, 8889);  // Hub's action response port
        udp.print(responseJson);
        udp.endPacket();

        return responseMessage;
    }
    
    // Action not found
    StaticJsonDocument<512> responseDoc;
    responseDoc["type"] = "action_response";
    responseDoc["id"] = payload["id"] | "unknown";
    responseDoc["status"] = "error";
    responseDoc["message"] = "Action not found";
    responseDoc["timestamp"] = millis() / 1000;

    String responseJson;
    serializeJson(responseDoc, responseJson);
    
    udp.beginPacket(BROADCAST_IP, 8889);  // Hub's action response port
    udp.print(responseJson);
    udp.endPacket();

    return "Action not found";
}

void DeviceHub::savePersistentData(const char* key, const String& value) {
    prefs.begin("device-hub", false);
    prefs.putString(key, value);
    prefs.end();
}

String DeviceHub::loadPersistentData(const char* key, const String& defaultValue) {
    prefs.begin("device-hub", false);
    String value = prefs.getString(key, defaultValue);
    prefs.end();
    return value;
}

void DeviceHub::registerEvent(const String& eventId, const String& eventName, const JsonObject& dataSchema) {
    Event event;
    event.id = eventId;
    event.name = eventName;
    event.dataSchema = dataSchema;
    events[eventId] = event;
}

void DeviceHub::emitEvent(const String& eventId, const JsonObject& payload) {
    if (events.find(eventId) == events.end()) {
        Serial.printf("Event %s not registered\n", eventId.c_str());
        return;
    }

    StaticJsonDocument<512> doc;
    doc["type"] = "device_event";
    doc["version"] = API_VERSION;
    doc["event"] = eventId;
    doc["uuid"] = deviceUUID;
    doc["timestamp"] = millis() / 1000;
    doc["payload"] = payload;

    String jsonString;
    serializeJson(doc, jsonString);
    sendUdpMessage(jsonString);
}

bool DeviceHub::isUUIDValid() {
    return deviceUUID.length() == 36 && 
           deviceUUID[8] == '-' && 
           deviceUUID[13] == '-' && 
           deviceUUID[18] == '-' && 
           deviceUUID[23] == '-';
}

void DeviceHub::setDeviceUUID(const String& uuid) {
    // Validate UUID format
    if (uuid.length() == 36 && 
        uuid[8] == '-' && 
        uuid[13] == '-' && 
        uuid[18] == '-' && 
        uuid[23] == '-') {
        
        deviceUUID = uuid;
        
        // Optionally store the manually set UUID
        prefs.begin("devicehub", false);
        prefs.putString("device_uuid", deviceUUID);
        prefs.end();
    } else {
        Serial.println("Invalid UUID format. UUID not set.");
    }
}

void DeviceHub::ensureDeviceUUID() {
    // First, check if a UUID was manually set
    if (isUUIDValid()) {
        return;
    }
    
    // If not, try to load from persistent storage
    prefs.begin("devicehub", false);
    deviceUUID = prefs.getString("device_uuid", "");
    prefs.end();
    
    // If still not valid, generate a new UUID
    if (!isUUIDValid()) {
        generateAndStoreUUID();
    }
}

void DeviceHub::generateAndStoreUUID() {
    // Generate version 4 UUID (random)
    uint32_t uuidParts[4];
    
    // Get random bytes
    for (int i = 0; i < 4; i++) {
        uuidParts[i] = esp_random();
    }
    
    // Format as UUID string (8-4-4-4-12)
    char uuidStr[37];
    snprintf(uuidStr, sizeof(uuidStr),
        "%08x-%04x-%04x-%04x-%04x%08x",
        (unsigned int)(uuidParts[0]),
        (unsigned int)(uuidParts[1] >> 16) & 0xFFFF,
        (unsigned int)(uuidParts[1] & 0xFFFF),
        (unsigned int)(uuidParts[2] >> 16) & 0xFFFF,
        (unsigned int)(uuidParts[2] & 0xFFFF),
        (unsigned int)(uuidParts[3])
    );
    
    // Set version bits (4) and variant bits (8,9)
    uuidStr[14] = '4';
    uuidStr[19] = (uuidStr[19] & 0x3F) | 0x80;
    
    deviceUUID = String(uuidStr);
    
    // Store in persistent storage
    prefs.begin("devicehub", false);
    prefs.putString("device_uuid", deviceUUID);
    prefs.end();
}

void DeviceHub::sendPeriodicUpdate() {
    static unsigned long lastUpdate = 0;
    unsigned long currentMillis = millis();
    if (currentMillis - lastUpdate >= 15000) {  // Send update every 5 minutes
        lastUpdate = currentMillis;
        sendDeviceInfo();
    }
}

void DeviceHub::registerEmergencyAction(ActionCallback callback) {
    emergencyAction = callback;
}

void DeviceHub::registerResetAction(ActionCallback callback) {
    resetAction = callback;
}

void DeviceHub::sendDeviceInfo() {
    String deviceInfo = getDeviceInfo();
    sendUdpMessage(deviceInfo);
}
