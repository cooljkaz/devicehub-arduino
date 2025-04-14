#include "DeviceHub.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <Preferences.h>

Preferences prefs;
const IPAddress DeviceHub::BROADCAST_IP(255, 255, 255, 255);
const char* DeviceHub::API_VERSION = DEVICEHUB_VERSION_2;

// UDP instances only
WiFiUDP udp;

// Forward declaration for the helper function
String getPathFromPacket(CoapPacket& packet);

DeviceHub::DeviceHub(const char* ssid, const char* password, const char* deviceName, const char* deviceType)
    : ssid(ssid), password(password), deviceName(deviceName), deviceType(deviceType), 
      currentState(DeviceState::Normal), dtlsEnabled(false),
      coap(nullptr), secureCoap(nullptr),
      debugSerial(nullptr), debugEnabled(false) {
    prefs.begin("devicehub", false);
}

DeviceHub::~DeviceHub() {
    log("Cleaning up DeviceHub...");
    delete coap;
    if (dtlsEnabled) {
        delete secureCoap;
    }
    log("DeviceHub cleanup complete.");
}

void DeviceHub::begin() {
    begin(Serial);
}

void DeviceHub::begin(HardwareSerial& serial) {
    debugSerial = &serial;
    debugEnabled = true;
    log("Initializing DeviceHub...");
    
    connectWiFi();
    
    // --- Revert UDP Begin --- 
    /*
    log("Explicitly starting UDP listener on port %d...", COAP_PORT);
    if (!udp.begin(COAP_PORT)) { 
        log("Error: Failed to bind main UDP socket on port %d!", COAP_PORT);
        return; 
    } else {
        log("Main UDP socket successfully bound to port %d.", COAP_PORT);
    }
    */
    // ------------------------

    // Initialize CoAP server
    log("Creating CoAP instance...");
    coap = new Coap(udp, 1500);
    log("Starting CoAP instance on port %d...", COAP_PORT); // Pass port again
    if (!coap->start(COAP_PORT)) { // Pass port to start
        log("Error: Failed to start CoAP server instance!");
        return;
    } else {
        log("CoAP server instance started successfully.");
    }

    // Initialize Secure CoAP if needed
    if (dtlsEnabled) {
        log("Creating/Starting Secure CoAP instance...");
        secureCoap = new Coap(udp, 1500); // Using main UDP for now
        if (!secureCoap->start(SECURE_COAP_PORT)) { 
            log("Error: Failed to start Secure CoAP instance!");
            // Consider returning or just logging
        } else {
            log("Secure CoAP server instance started successfully.");
        }
    }
    
    // Setup CoAP resources
    setupCoAPResources();
    
    // Initialize other features
    ensureDeviceUUID();
    otaHelper.start(ssid, password, deviceName, password, 3232, 115200);
    
    log("DeviceHub initialized successfully");
}

void DeviceHub::enableDebug(HardwareSerial& serial) {
    debugSerial = &serial;
    debugEnabled = true;
    log("Debug logging enabled");
}

void DeviceHub::disableDebug() {
    debugEnabled = false;
    debugSerial = nullptr;
}

void DeviceHub::log(const char* format, ...) {
    if (!debugEnabled || !debugSerial) return;
    
    va_list args;
    va_start(args, format);
    char buffer[256];
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    debugSerial->printf("[DeviceHub] %s\n", buffer);
}

void DeviceHub::loop() {
    if (WiFi.status() != WL_CONNECTED) {
        log("WiFi disconnected. Attempting reconnect...");
        connectWiFi();
        if (WiFi.status() != WL_CONNECTED) return; 
    }
    
    // Handle CoAP messages
    if (coap) {
        coap->loop();
    } else {
        log("Warning: coap instance is null in loop.");
    }
    
    // Secure CoAP loop
    if (dtlsEnabled && secureCoap) {
        secureCoap->loop();
    } else if (dtlsEnabled && !secureCoap) {
         log("Warning: dtlsEnabled but secureCoap instance is null in loop.");
    }
    
    // --- Temporarily comment out other loop tasks --- 
    
    resendPendingMessages();
    sendPeriodicUpdate(); // This calls sendDeviceInfo
    otaHelper.handle();
    
    // ---------------------------------------------
}

void DeviceHub::setupCoAPResources() {
    log("Setting up CoAP server resources (Simplified Approach)...");

    // --- Handler for POST /v2/actions/resetPuzzle --- 
    coap->server([this](CoapPacket& packet, IPAddress ip, int port) {
        log("Callback fired for path: v2/actions");
        if (packet.code == COAP_POST) {
            log("Handling POST /v2/actions request...");
            StaticJsonDocument<512> doc;
            DeserializationError error = deserializeJson(doc, packet.payload, packet.payloadlen);
            
            log("Payload: %s", doc.as<String>().c_str());

            if (error) {
                log("JSON deserialization failed: %s", error.c_str());
                sendCoAPResponse(ip, port, packet.messageid, "Error: Invalid JSON payload.", 
                                 COAP_BAD_REQUEST, COAP_TEXT_PLAIN, 
                                 packet.token, packet.tokenlen);
            } else {
                String result = processRequest(doc.as<JsonObject>());
                sendCoAPResponse(ip, port, packet.messageid, result, 
                            COAP_CHANGED, COAP_TEXT_PLAIN, 
                            packet.token, packet.tokenlen);
            }
        } else {
            log("Method %d not allowed for /v2/actions/resetPuzzle. Sending 4.05", packet.code);
            sendCoAPResponse(ip, port, packet.messageid, "", 
                             COAP_METHOD_NOT_ALLOWD, COAP_NONE, 
                             packet.token, packet.tokenlen);
        }
        log("Finished handling v2/actions request.");
    }, "v2/actions"); // Register specific path

    // Default response handler (for ACKs to our requests etc.)
    coap->response([this](CoapPacket& packet, IPAddress ip, int port) {
        log("Received CoAP ACK/Response (Message ID: %d, Type: %d)", packet.messageid, packet.type);
    });
    
    log("CoAP server resources set up (Simplified: Specific Paths).");
}

String DeviceHub::processRequest(const JsonObject& doc) {
    if (doc.containsKey("action")) {
        String actionId = doc["action"].as<String>();
        JsonObject payload = doc["payload"].as<JsonObject>();
        return handleAction(actionId, payload);
    }
    return "Invalid request";
}

String DeviceHub::handleAction(const String& actionId, const JsonObject& payload) {
    log("Handling action: %s", actionId.c_str());
    if (actions.find(actionId) != actions.end()) {
        String response = actions[actionId].callback(payload);
        // Send action response event
        StaticJsonDocument<1024> doc;
        doc["type"] = "action_response";
        doc["uuid"] = deviceUUID;
        doc["action"] = actionId;
        doc["status"] = "success";
        doc["version"] = DEVICEHUB_VERSION_2;
        String output;
        serializeJson(doc, output);
        
        return output;
    }
    return "Action not found";
}

void DeviceHub::notifyObservers(const String& eventId, const JsonObject& payload, const String& version) {
    for (const auto& observer : observers) {
        if (observer.version == version) {
            // Send notification to observer
            StaticJsonDocument<1024> doc;
            doc["event"] = eventId;
            doc["payload"] = payload;
            doc["timestamp"] = millis();
            String output;
            serializeJson(doc, output);
            coap->sendResponse(observer.ip, observer.port, 0, output.c_str());
        }
    }
}

void DeviceHub::sendCoAPResponse(IPAddress ip, int port, uint16_t messageId, 
                                 const String& payload, 
                                 COAP_RESPONSE_CODE code, 
                                 COAP_CONTENT_TYPE type, 
                                 const uint8_t *token, 
                                 int tokenlen)
{
    log("Sending CoAP Response: Code=%d.%02d, Type=%d, MsgID=%d, TokenLen=%d, PayloadLen=%d",
        (code >> 5), (code & 0x1F), type, messageId, tokenlen, payload.length());
        
    // Use the full overload of the library's sendResponse function
    uint16_t sendResult = coap->sendResponse(ip, port, messageId, 
                                             payload.c_str(), payload.length(), 
                                             code, 
                                             type, 
                                             token, 
                                             tokenlen);

    if (sendResult == 0) {
        log("Error: coap->sendResponse returned 0 (failed to send).");
    } else {
        log("coap->sendResponse called successfully. Result: %d", sendResult);
    }
}

String DeviceHub::getSupportedVersions() {
    StaticJsonDocument<256> doc;
    JsonArray versions = doc.createNestedArray("versions");
    versions.add(DEVICEHUB_VERSION_2);
    
    String output;
    serializeJson(doc, output);
    return output;
}

bool DeviceHub::isVersionSupported(const String& version) {
    return version == DEVICEHUB_VERSION_2;
}

String DeviceHub::getDeviceInfo() {
    log("Generating full device info...");
    
    // Use a larger document size temporarily to measure the full potential size
    // We won't serialize from this if it's too big for the CoAP buffer.
    StaticJsonDocument<2048> doc; 
    
    doc["type"] = "device_info";
    doc["version"] = API_VERSION;
    doc["uuid"] = deviceUUID;
    doc["name"] = deviceName;
    doc["deviceType"] = deviceType;

    JsonObject capabilities = doc.createNestedObject("capabilities");
    JsonArray actionsArray = capabilities.createNestedArray("actions");
    for (const auto& actionPair : actions) {
        const Action& action = actionPair.second;
        JsonObject actionObj = actionsArray.createNestedObject();
        actionObj["id"] = action.id;
        actionObj["name"] = action.name;
        
        // Re-enable parameters
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

    JsonArray eventsArray = capabilities.createNestedArray("events");
    for (const auto& eventPair : events) {
        const Event& event = eventPair.second;
        JsonObject eventObj = eventsArray.createNestedObject();
        eventObj["id"] = event.id;
        eventObj["name"] = event.name;
        // Re-enable schema
        eventObj["dataSchema"] = event.dataSchema; 
    }
    
    JsonObject status = doc.createNestedObject("status");
    switch (currentState) {
        case DeviceState::Normal: status["state"] = "normal"; break;
        case DeviceState::Emergency: status["state"] = "emergency"; break;
        case DeviceState::Idle: status["state"] = "idle"; break;
        case DeviceState::Active: status["state"] = "active"; break;
        case DeviceState::Error: status["state"] = "error"; break;
    }

    // --- Size Check --- 
    size_t jsonPayloadSize = measureJson(doc);
    const size_t COAP_OVERHEAD_ESTIMATE = 50; // Estimated bytes for headers/options
    const size_t COAP_BUFFER_LIMIT = 1500;    // Updated limit to match Coap constructor
    size_t estimatedTotalPacketSize = jsonPayloadSize + COAP_OVERHEAD_ESTIMATE;

    log("Full device info JSON payload size: %d bytes", jsonPayloadSize);
    log("Estimated total CoAP packet size: %d bytes (Limit: %d)", estimatedTotalPacketSize, COAP_BUFFER_LIMIT);

    String jsonString;
    if (estimatedTotalPacketSize <= COAP_BUFFER_LIMIT) {
        // Safe to serialize and return the full info
        serializeJson(doc, jsonString);
        log("Full device info fits within CoAP buffer (%d bytes).", COAP_BUFFER_LIMIT);
    } else {
        // Payload too large, generate a minimal truncated message
        log("Error: Full device info exceeds CoAP buffer limit (%d bytes). Sending truncated info.", COAP_BUFFER_LIMIT);
        StaticJsonDocument<256> truncatedDoc; // Small buffer for fallback
        truncatedDoc["type"] = "device_info_truncated";
        truncatedDoc["version"] = API_VERSION;
        truncatedDoc["uuid"] = deviceUUID;
        truncatedDoc["name"] = deviceName;
        truncatedDoc["deviceType"] = deviceType;
        truncatedDoc["error"] = "Full info too large";
        serializeJson(truncatedDoc, jsonString);
    }
    
    log("Device info generation complete.");
    return jsonString;
}

void DeviceHub::sendDeviceInfo() {
    log("Starting sendDeviceInfo process...");
    String deviceInfo = getDeviceInfo(); // Gets full or truncated info based on size check
    
    if (!coap) {
        log("Error: CoAP instance is null in sendDeviceInfo.");
        return;
    }

    log("Attempting to send device info via CoAP (NON-CONFIRMABLE - Payload size: %d)...", deviceInfo.length());
    
    // Send the CoAP message (full or truncated)
    uint16_t messageId = coap->send(BROADCAST_IP, COAP_PORT, "device/info", 
                                    COAP_NONCON,
                                    COAP_POST, NULL, 0,
                                    (uint8_t*)deviceInfo.c_str(), deviceInfo.length(),
                                    COAP_APPLICATION_JSON);
    
    if (messageId == 0) {
        log("Error: CoAP send function returned 0 (failed to send).");
    } else {
        log("CoAP send function called successfully (NON-CONFIRMABLE). Message ID: %d", messageId);
    }

    log("Completed sendDeviceInfo process.");
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
    
    log("Sending message of type %s: %s", type.c_str(), message.c_str());
    sendCoAPMessage(jsonString);
}

void DeviceHub::sendCoAPMessage(const String& payload, COAP_TYPE messageType) {
    if (!coap) {
        log("Error: CoAP instance is null in sendCoAPMessage.");
        return;
    }
    log("Attempting to send CoAP message (Type: %d, Payload size: %d)...", messageType, payload.length());
    uint16_t messageId = coap->send(BROADCAST_IP, COAP_PORT, "message", 
                                    messageType, COAP_POST, NULL, 0,
                                    (uint8_t*)payload.c_str(), payload.length(),
                                    COAP_APPLICATION_JSON);

    if (messageId == 0) {
        log("Error: CoAP send function returned 0 (failed to send) for type %d.", messageType);
    } else {
        log("CoAP send function called successfully (Type: %d). Message ID: %d", messageType, messageId);
    }
}

void DeviceHub::sendCoAPMessage(const String& payload) {
    // Defaults to COAP_CON for general messages unless specified otherwise
    sendCoAPMessage(payload, COAP_CON); 
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
    log("Registered action: %s", actionId.c_str());
}

void DeviceHub::registerAction(const String& actionId, const String& actionName, ActionCallback callback) {
    Action action;
    action.id = actionId;
    action.name = actionName;
    action.callback = callback;
    actions[actionId] = action;
    log("Registered action: %s (%s)", actionId.c_str(), actionName.c_str());
}

void DeviceHub::registerAction(const String& actionId, const String& actionName, ActionCallback callback, const std::vector<ActionParameter>& parameters) {
    Action action;
    action.id = actionId;
    action.name = actionName;
    action.parameters = parameters;
    action.callback = callback;
    actions[actionId] = action;
    log("Registered action: %s (%s) with %d parameters", actionId.c_str(), actionName.c_str(), parameters.size());
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
    if (currentMillis - lastUpdate >= 15000) {  // Send update every 15 seconds
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

void DeviceHub::connectWiFi() {
    log("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) { // Limit attempts
        delay(500);
        // Use the log method if enabled, otherwise print dots
        if (debugEnabled && debugSerial) {
            debugSerial->print("."); 
        } else {
            Serial.print(".");
        }
        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        log("\nConnected to WiFi. IP: %s", WiFi.localIP().toString().c_str());
    } else {
        log("\nFailed to connect to WiFi after %d attempts.", attempts);
        // Handle connection failure? Maybe retry later or enter an error state?
    }
}

void DeviceHub::handleEmergencyStart() {
    if (currentState != DeviceState::Emergency) {
        log("Entering emergency state...");
        currentState = DeviceState::Emergency;
        if (emergencyAction) {
            log("Executing emergency action...");
            String result = emergencyAction(JsonObject()); // Assuming no payload needed
            log("Emergency action result: %s", result.c_str());
            // Maybe send an emergency message here?
        } else {
            log("No emergency action registered.");
        }
    }
}

void DeviceHub::handleEmergencyEnd() {
    if (currentState == DeviceState::Emergency) {
        log("Exiting emergency state...");
        currentState = DeviceState::Normal;
        if (resetAction) {
            log("Executing reset action...");
            String result = resetAction(JsonObject()); // Assuming no payload needed
            log("Reset action result: %s", result.c_str());
        } else {
            log("No reset action registered.");
        }
    }
}

void DeviceHub::emitEvent(const String& eventId, const JsonObject& payload) {
    log("Attempting to emit event: %s", eventId.c_str());

    // --- Check if the event is registered --- 
    if (events.find(eventId) == events.end()) {
        log("Error: Cannot emit unregistered event '%s'. Register it first using registerEvent().", eventId.c_str());
        return; // Do not proceed if event is not registered
    }
    // ----------------------------------------
    
    // Retrieve the registered event details (optional, could be used for validation)
    // const Event& registeredEvent = events[eventId];
    log("Event '%s' is registered. Proceeding with emission.", eventId.c_str());
    
    StaticJsonDocument<1024> doc;
    doc["type"] = "device_event";
    doc["uuid"] = deviceUUID;
    doc["event"] = eventId;
    doc["version"] = DEVICEHUB_VERSION_2;
    
    // Check if payload is null before adding
    if (!payload.isNull()) {
        doc["data"] = payload; 
    } else {
        doc["data"] = JsonObject(); 
    }
    
    String output;
    size_t len = measureJson(doc);
    log("Event JSON size: %d bytes", len);

    if (len > 1024) {
         log("Warning: Event JSON exceeds buffer size (1024). Event '%s' not sent.", eventId.c_str());
         return; // Skip sending if too large
    }

    serializeJson(doc, output);
    
    log("Sending event JSON: %s", output.c_str());
    // Use NON_CON since events are typically just broadcasts
    sendCoAPMessage(output, COAP_NONCON); 
    
    // notifyObservers(eventId, payload, DEVICEHUB_VERSION_2); // Keep observer logic if needed?
}

