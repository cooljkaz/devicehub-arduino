#include "DeviceHub.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include <nvs_flash.h>
#include <esp_err.h>
#include <algorithm>

// NOTE: We avoid global object constructors that could crash on ESP32-S3
// These are allocated dynamically in begin() instead
Preferences* prefsPtr = nullptr;
WiFiUDP* udpPtr = nullptr;

// Static members
// NOTE: On ESP32-S3, global IPAddress constructors can crash before Arduino runtime initializes.
// We use a lazy-initialization pattern for these IP addresses.
static IPAddress* broadcastIPPtr = nullptr;
static IPAddress* coapMulticastIPPtr = nullptr;

// Lazy-initialized getter for BROADCAST_IP
static IPAddress& getBroadcastIP() {
    if (!broadcastIPPtr) {
        broadcastIPPtr = new IPAddress(255, 255, 255, 255);
    }
    return *broadcastIPPtr;
}

// Lazy-initialized getter for COAP_MULTICAST_IP
static IPAddress& getCoapMulticastIP() {
    if (!coapMulticastIPPtr) {
        coapMulticastIPPtr = new IPAddress(224, 0, 1, 187);
    }
    return *coapMulticastIPPtr;
}

// These must be defined but are not used directly - use the getter functions instead
const IPAddress DeviceHub::BROADCAST_IP(0, 0, 0, 0);  // Placeholder - use getBroadcastIP()
const IPAddress DeviceHub::COAP_MULTICAST_IP(0, 0, 0, 0);  // Placeholder - use getCoapMulticastIP()
const char* DeviceHub::API_VERSION = DEVICEHUB_VERSION_2;

// Forward declaration for the helper function
String getPathFromPacket(CoapPacket& packet);

// Single constructor - uses networks configured via build defines
// NOTE: We avoid heavy initialization here (NVS, WiFi, etc.) because global constructors
// run before Arduino's setup() and can crash on ESP32-S3 and other variants.
DeviceHub::DeviceHub(const char* deviceName)
    : deviceName(deviceName), deviceType("esp32"),
      currentState(DeviceState::Normal), dtlsEnabled(false),
      coap(nullptr), secureCoap(nullptr),
      debugSerial(nullptr), debugEnabled(false),
      apPassword(DEFAULT_AP_PASSWORD), wifiTimeoutMs(DEFAULT_WIFI_TIMEOUT_MS), enableAPFallback(false),
      currentWiFiMode(WiFiMode::Offline), lastConnectionCheck(0), lastSignalCheck(0),
      connectionCheckIntervalMs(DEFAULT_CONNECTION_CHECK_INTERVAL_MS),
      minSignalStrength(DEFAULT_MIN_SIGNAL_STRENGTH), currentSignalStrength(-100),
      apModeForced(false), totalReconnectionAttempts(0), successfulReconnections(0), openAPMode(DEFAULT_AP_OPEN_NETWORK),
      currentEnvironment(WiFiEnvironment::Unknown), useMultiWiFi(true),
      productionCheckIntervalMs(300000), lastProductionCheck(0),
      forceSingleCore(false), isDualCore(false), detectedCores(1),
      networkTaskHandle(nullptr), commandQueue(nullptr), statusQueue(nullptr), statusMutex(nullptr),
      networkTaskRunning(false), lastNetworkTaskHealthCheck(0),
      cachedWiFiMode(WiFiMode::Offline), cachedConnected(false), cachedSignalStrength(-100), cachedIsAPMode(false),
      bleBeaconConfigured(false), bleBeaconActive(false),
      bleBeaconMajor(0), bleBeaconMinor(0), bleBeaconTxPower(-59),
      firmwareVersion("0.0.0"), otaUpdatePending(false) {
    // MINIMAL constructor - do nothing here that could crash on ESP32-S3
    // All initialization is deferred to begin() which runs during setup()
    // after the Arduino runtime is fully initialized.
}

// Configuration setters (call before begin())
void DeviceHub::setDeviceType(const char* type) {
    deviceType = type;
    log("Device type set to: %s", type);
}

void DeviceHub::setEnableAPFallback(bool enable) {
    enableAPFallback = enable;
    log("AP fallback %s", enable ? "enabled" : "disabled");
}

void DeviceHub::setAPPassword(const char* password) {
    if (strlen(password) >= 8) {
        apPassword = password;
        log("AP password set");
    } else {
        log("Warning: AP password too short (min 8 chars), using default");
        apPassword = DEFAULT_AP_PASSWORD;
    }
}

void DeviceHub::setOpenAP(bool open) {
    openAPMode = open;
    log("Open AP mode %s", open ? "enabled" : "disabled");
}

void DeviceHub::setWiFiTimeout(int timeoutMs) {
    if (timeoutMs >= 5000) {
        wifiTimeoutMs = timeoutMs;
        log("WiFi timeout set to %d ms", timeoutMs);
    } else {
        log("Warning: WiFi timeout too short (min 5000ms)");
    }
}

void DeviceHub::setForceSingleCore(bool force) {
    forceSingleCore = force;
    log("Force single core: %s", force ? "yes" : "no");
}

void DeviceHub::setProductionCheckInterval(unsigned long intervalMs) {
    if (intervalMs >= 60000) {  // Minimum 1 minute
        productionCheckIntervalMs = intervalMs;
        log("Production check interval set to %lu ms", intervalMs);
    } else {
        log("Warning: Production check interval too short (min 60000ms)");
    }
}

void DeviceHub::setFirmwareVersion(const char* version) {
    firmwareVersion = version;
    log("Firmware version set to: %s", version);
}

void DeviceHub::setBLEBeacon(uint16_t major, uint16_t minor, int8_t txPower) {
    bleBeaconMajor = major;
    bleBeaconMinor = minor;
    bleBeaconTxPower = txPower;
    bleBeaconConfigured = true;
    log("BLE iBeacon configured: Major=%d, Minor=%d, TxPower=%d", major, minor, txPower);
}

DeviceHub::~DeviceHub() {
    log("Cleaning up DeviceHub...");

    // Stop BLE beacon if active
    if (bleBeaconActive) {
        stopBLEBeacon();
    }

    // Stop network task if running
    if (isDualCore && networkTaskRunning) {
        stopNetworkTask();
    }
    
    // Cleanup FreeRTOS objects
    cleanupFreeRTOSObjects();
    
    // Cleanup CoAP instances
    delete coap;
    if (dtlsEnabled) {
        delete secureCoap;
    }
    
    log("DeviceHub cleanup complete.");
}

void DeviceHub::begin() {
    begin(Serial);
}

void DeviceHub::begin(Print& serial) {
    debugSerial = &serial;
    debugEnabled = true;
    log("DeviceHub::begin() called - Initializing core systems...");

    // Allocate global objects now (deferred from static initialization for ESP32-S3 compatibility)
    if (!prefsPtr) {
        prefsPtr = new Preferences();
    }
    if (!udpPtr) {
        udpPtr = new WiFiUDP();
    }

    // Initialize NVS (Non-Volatile Storage) first - this is critical for Preferences
    initializeNVS();

    // Generate AP SSID and mDNS hostname (deferred from constructor for ESP32-S3 compatibility)
    apSSID = generateAPSSID();
    mdnsHostname = generateMDNSHostname();
    log("AP SSID: %s, mDNS: %s", apSSID.c_str(), mdnsHostname.c_str());

    // Initialize network status struct
    currentNetworkStatus.wifiMode = WiFiMode::Offline;
    currentNetworkStatus.connected = false;
    currentNetworkStatus.signalStrength = -100;
    currentNetworkStatus.lastUpdate = 0;
    currentNetworkStatus.isAPMode = false;

    // Auto-configure networks from build defines
    autoConfigureNetworks();

    // NOTE: Do NOT call esp_netif_init() or esp_event_loop_create_default() here!
    // The Arduino ESP32 WiFi library handles TCP/IP and event loop initialization internally.
    // Calling them manually can cause conflicts or double-initialization crashes on ESP32-S3.

    // NOTE: UDP sockets cannot be created until after WiFi.begin() initializes the TCP/IP stack.
    // Emergency UDP will be initialized after WiFi connection is established.

    // Detect and setup dual-core if available
    if (detectAndSetupDualCore()) {
        log("Dual-core mode activated - Network operations on Core %d, Application on Core %d", 
            DEVICEHUB_NETWORK_CORE, DEVICEHUB_APPLICATION_CORE);
        startNetworkTask();
        delay(100); // Brief delay for network task init
    } else {
        log("Single-core mode - All operations on main core");
        // ONLY perform WiFi init here if NOT using dual core (network task will handle it otherwise)
        if (!isDualCore) { // This check is technically redundant if detectAndSetupDualCore() sets isDualCore correctly
            if (apModeForced) {
                log("AP mode forced, setting up Access Point...");
                setupAccessPoint(); // Uses Arduino WiFi API
            } else if (attemptWiFiConnection()) { // Uses Arduino WiFi API
                log("WiFi connection successful, setting up Station mode...");
                setupStationMode(); // Uses Arduino WiFi API
            } else if (enableAPFallback) {
                log("WiFi connection failed, falling back to Access Point mode...");
                setupAccessPoint(); // Uses Arduino WiFi API
            } else {
                log("WiFi connection failed and AP fallback disabled. Operating offline.");
                currentWiFiMode = WiFiMode::Offline;
            }
            if (currentWiFiMode != WiFiMode::Offline) {
                setupMDNS();
            }
        }
    }
    
    log("Creating CoAP instance...");
    coap = new Coap(*udpPtr, 1500);
    log("Starting CoAP instance on port %d...", COAP_PORT);
    if (!coap->start(COAP_PORT)) {
        log("Error: Failed to start CoAP server instance!");
        return;
    } else {
        log("CoAP server instance started successfully.");
    }

    if (dtlsEnabled) {
        log("Creating/Starting Secure CoAP instance...");
        secureCoap = new Coap(*udpPtr, 1500);
        if (!secureCoap->start(SECURE_COAP_PORT)) { 
            log("Error: Failed to start Secure CoAP instance!");
        } else {
            log("Secure CoAP server instance started successfully.");
        }
    }
    
    setupCoAPResources();
    ensureDeviceUUID();

    // Register built-in firmware OTA action
    registerAction("updateFirmware", "Update Firmware", [this](const JsonObject& payload) {
        String url = payload["url"] | "";
        if (url.length() == 0) return String("No URL provided");
        pendingOTAUrl = url;
        otaUpdatePending = true;
        log("OTA update queued from URL: %s", url.c_str());
        return String("Update queued, downloading...");
    });

    // Only start OTA helper if we have WiFi connection (not AP-only mode)
    if (!isAPOnlyMode() && !configuredNetworks.empty() && currentWiFiMode == WiFiMode::Station) {
        log("Starting OTA helper...");
        // Use the connected network's credentials for OTA
        for (const auto& network : configuredNetworks) {
            if (network.ssid == connectedSSID) {
                otaHelper.start(network.ssid.c_str(), network.password.c_str(), deviceName, network.password.c_str(), 3232, 115200);
                break;
            }
        }
    } else {
        log("Skipping OTA helper (AP-only mode or no WiFi connection)");
    }
    
    log("DeviceHub initialized successfully");
    
    // Print connection status
    if (isDualCore) {
        log("Running in DUAL-CORE mode:");
        log("- Network Task: Core %d (Priority %d, Stack %d bytes)", 
            DEVICEHUB_NETWORK_CORE, DEVICEHUB_NETWORK_TASK_PRIORITY, DEVICEHUB_NETWORK_TASK_STACK_SIZE);
        log("- Application: Core %d (Main Loop)", DEVICEHUB_APPLICATION_CORE);
        log("- Network Task Running: %s", networkTaskRunning ? "Yes" : "No");
    } else {
        log("Running in SINGLE-CORE mode (Core %d)", xPortGetCoreID());
    }
    
    // Update cached status for dual-core mode
    if (isDualCore) {
        // Wait a moment for network task to report status
        delay(200);
    }
    
    // Print current network status
    WiFiMode mode = isDualCore ? cachedWiFiMode : currentWiFiMode;
    if (mode == WiFiMode::Station) {
        IPAddress ip = isDualCore ? cachedIP : WiFi.localIP();
        int signal = isDualCore ? cachedSignalStrength : getSignalStrength();
        log("Mode: WiFi Station - IP: %s - mDNS: http://%s.local", 
            ip.toString().c_str(), mdnsHostname.c_str());
        log("Signal: %d dBm", signal);
    } else if (mode == WiFiMode::AccessPoint) {
        IPAddress ip = isDualCore ? cachedIP : WiFi.softAPIP();
        log("Mode: WiFi Access Point - SSID: %s - Password: %s", apSSID.c_str(), apPassword);
        log("IP: %s - mDNS: http://%s.local", ip.toString().c_str(), mdnsHostname.c_str());
        log("Connect to the '%s' network and access the device at http://%s.local", 
            apSSID.c_str(), mdnsHostname.c_str());
    }
}

void DeviceHub::enableDebug(Print& serial) {
    debugSerial = &serial;
    debugEnabled = true;
    log("Debug logging enabled");
}

void DeviceHub::disableDebug() {
    debugEnabled = false;
    debugSerial = nullptr;
}

// Converts "DMX Controller"  →  "dmx-controller"
String DeviceHub::toMdnsHost(String in) {
    in.toLowerCase();
    in.replace(" ", "-");
    for (size_t i = 0; i < in.length(); ++i) {
        if (!isalnum(in[i]) && in[i] != '-') in.remove(i--, 1); // strip bad chars
    }
    return in;
}

void DeviceHub::startMDNS() {
    // This method is now primarily called by setupMDNS()
    // Keep for backward compatibility but redirect to new implementation
    if (currentWiFiMode != WiFiMode::Offline) {
        setupMDNS();
    } else {
        log("mDNS not started – no network connection available");
    }
}

void DeviceHub::log(const char* format, ...) {
    if (!debugEnabled || !debugSerial) return;
    
    va_list args;
    va_start(args, format);
    char buffer[256];
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    debugSerial->print("[DeviceHub] ");
    debugSerial->println(buffer);
}

void DeviceHub::loop() {
    // Check for pending OTA update (deferred so CoAP ACK goes out first)
    if (otaUpdatePending) {
        otaUpdatePending = false;
        performOTAUpdate(pendingOTAUrl.c_str());
        // performOTAUpdate calls ESP.restart() on success, so we only reach here on failure
    }

    if (isDualCore) {
        // Dual-core mode: lightweight main loop

        // Check network task health
        checkNetworkTaskHealth();
        
        // Update cached status from network task
        NetworkStatus status;
        if (statusQueue && xQueueReceive(statusQueue, &status, 0) == pdTRUE) {
            updateStatusFromNetworkTask(status);
        }
        
        // Handle CoAP messages (keep on main core for responsiveness)
        if (coap) {
            coap->loop();
        }
        
        // Secure CoAP loop
        if (dtlsEnabled && secureCoap) {
            secureCoap->loop();
        }
        
        // Handle other main-core operations
        handleEmergencyPacket();
        otaHelper.handle();
        
    } else {
        // Single-core mode: traditional full loop
        unsigned long currentTime = millis();
        
        // Immediate WiFi status check for station mode
        if (!apModeForced && currentWiFiMode == WiFiMode::Station) {
            if (WiFi.status() != WL_CONNECTED) {
                log("WiFi connection lost! Status: %d. Attempting immediate reconnection...", WiFi.status());
                totalReconnectionAttempts++;
                if (attemptWiFiConnection()) {
                    successfulReconnections++;
                    log("WiFi reconnected successfully! (Attempt #%lu, Success #%lu)", 
                        totalReconnectionAttempts, successfulReconnections);
                } else if (enableAPFallback) {
                    log("WiFi reconnection failed, switching to AP mode...");
                    setupAccessPoint();
                    setupMDNS();
                } else {
                    log("WiFi reconnection failed and AP fallback disabled.");
                }
            }
        }
        
        // Periodic comprehensive connection check
        if (currentTime - lastConnectionCheck > connectionCheckIntervalMs) {
            lastConnectionCheck = currentTime;
            checkAndReconnect();
        }
        
        // Periodic signal strength monitoring
        if (currentWiFiMode == WiFiMode::Station && 
            currentTime - lastSignalCheck > DEFAULT_SIGNAL_STRENGTH_CHECK_INTERVAL_MS) {
            lastSignalCheck = currentTime;
            monitorSignalStrength();
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
        
        handleEmergencyPacket();
        resendPendingMessages();
        sendPeriodicUpdate(); // This calls sendDeviceInfo
        otaHelper.handle();
    }
}

void DeviceHub::setupCoAPResources() {
    log("Setting up CoAP server resources (RFC 6690 Standard Discovery)...");

    // RFC 6690: Standard discovery endpoint /.well-known/core
    // Returns CoRE Link Format listing available resources
    coap->server([this](CoapPacket& packet, IPAddress ip, int port) {
        if (packet.code == COAP_GET) {
            log("Handling GET /.well-known/core from %s:%d", ip.toString().c_str(), port);
            String links = getWellKnownCore();
            log("Responding with CoRE Link Format: %s", links.c_str());
            sendCoAPResponse(ip, port, packet.messageid, links,
                           COAP_CONTENT, (COAP_CONTENT_TYPE)COAP_CONTENT_TYPE_LINK_FORMAT,
                           packet.token, packet.tokenlen);
        } else {
            sendCoAPResponse(ip, port, packet.messageid, "",
                           COAP_METHOD_NOT_ALLOWD, COAP_NONE,
                           packet.token, packet.tokenlen);
        }
    }, ".well-known/core");

    // Device info endpoint (GET /info) - returns full JSON device capabilities
    coap->server([this](CoapPacket& packet, IPAddress ip, int port) {
        if (packet.code == COAP_GET) {
            log("Handling GET /info from %s:%d", ip.toString().c_str(), port);
            String deviceInfo = getDeviceInfo();
            sendCoAPResponse(ip, port, packet.messageid, deviceInfo,
                           COAP_CONTENT, COAP_APPLICATION_JSON,
                           packet.token, packet.tokenlen);
        } else {
            sendCoAPResponse(ip, port, packet.messageid, "",
                           COAP_METHOD_NOT_ALLOWD, COAP_NONE,
                           packet.token, packet.tokenlen);
        }
    }, "info");

    // Action endpoint (POST /action) - standard short path
    coap->server([this](CoapPacket& packet, IPAddress ip, int port) {
        if (packet.code == COAP_POST) {
            log("Handling POST /action from %s:%d", ip.toString().c_str(), port);
            JsonDocument doc;
            DeserializationError error = deserializeJson(doc, packet.payload, packet.payloadlen);

            if (error) {
                log("JSON deserialization failed: %s", error.c_str());
                sendCoAPResponse(ip, port, packet.messageid, "{\"error\":\"Invalid JSON\"}",
                                 COAP_BAD_REQUEST, COAP_APPLICATION_JSON,
                                 packet.token, packet.tokenlen);
            } else {
                String result = processRequest(doc.as<JsonObject>());
                sendCoAPResponse(ip, port, packet.messageid, result,
                            COAP_CHANGED, COAP_APPLICATION_JSON,
                            packet.token, packet.tokenlen);
            }
        } else {
            sendCoAPResponse(ip, port, packet.messageid, "",
                             COAP_METHOD_NOT_ALLOWD, COAP_NONE,
                             packet.token, packet.tokenlen);
        }
    }, "action");

    // Legacy v2/actions endpoint (POST) - for backward compatibility during transition
    coap->server([this](CoapPacket& packet, IPAddress ip, int port) {
        log("Callback fired for path: v2/actions");
        if (packet.code == COAP_POST) {
            log("Handling POST /v2/actions request...");
            JsonDocument doc;
            DeserializationError error = deserializeJson(doc, packet.payload, packet.payloadlen);

            log("Payload: %s", doc.as<String>().c_str());

            if (error) {
                log("JSON deserialization failed: %s", error.c_str());
                sendCoAPResponse(ip, port, packet.messageid, "{\"error\":\"Invalid JSON\"}",
                                 COAP_BAD_REQUEST, COAP_APPLICATION_JSON,
                                 packet.token, packet.tokenlen);
            } else {
                String result = processRequest(doc.as<JsonObject>());
                sendCoAPResponse(ip, port, packet.messageid, result,
                            COAP_CHANGED, COAP_APPLICATION_JSON,
                            packet.token, packet.tokenlen);
            }
        } else {
            log("Method %d not allowed for /v2/actions. Sending 4.05", packet.code);
            sendCoAPResponse(ip, port, packet.messageid, "",
                             COAP_METHOD_NOT_ALLOWD, COAP_NONE,
                             packet.token, packet.tokenlen);
        }
        log("Finished handling v2/actions request.");
    }, "v2/actions");

    // Default response handler (for ACKs to our requests etc.)
    coap->response([this](CoapPacket& packet, IPAddress ip, int port) {
        log("Received CoAP ACK/Response (Message ID: %d, Type: %d)", packet.messageid, packet.type);
    });

    log("CoAP resources configured: /.well-known/core, /info, /action, /v2/actions");
}

String DeviceHub::processRequest(const JsonObject& doc) {
    if (!doc["action"].isNull()) {
        String actionId = doc["action"].as<String>();
        JsonObject payload = doc["payload"].as<JsonObject>();
        // Extract correlation ID if present (could be "id" or "correlationId")
        String correlationId = "";
        if (!doc["id"].isNull()) {
            correlationId = doc["id"].as<String>();
        } else if (!doc["correlationId"].isNull()) {
            correlationId = doc["correlationId"].as<String>();
        }
        return handleAction(actionId, payload, correlationId);
    }
    return "Invalid request";
}

String DeviceHub::handleAction(const String& actionId, const JsonObject& payload, const String& correlationId) {
    log("Handling action: %s (correlationId: %s)", actionId.c_str(), correlationId.c_str());
    if (actions.find(actionId) != actions.end()) {
        String response = actions[actionId].callback(payload);
        // Send action response event
        JsonDocument doc;
        doc["type"] = "action_response";
        doc["uuid"] = deviceUUID;
        doc["action"] = actionId;
        doc["status"] = "success";
        doc["data"] = response;  // Include the actual action response data
        doc["version"] = DEVICEHUB_VERSION_2;
        if (correlationId.length() > 0) {
            doc["id"] = correlationId;
        }
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
            JsonDocument doc;
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
    JsonDocument doc;
    JsonArray versions = doc["versions"].to<JsonArray>();
    versions.add(DEVICEHUB_VERSION_2);
    
    String output;
    serializeJson(doc, output);
    return output;
}

bool DeviceHub::isVersionSupported(const String& version) {
    return version == DEVICEHUB_VERSION_2;
}

// NOTE: handleIncomingPacket() removed - discovery now uses standard CoAP multicast
// Devices respond to GET /.well-known/core requests instead of UDP port 8888

String DeviceHub::getDeviceInfo() {
    log("Generating full device info...");
    
    // Use a larger document size temporarily to measure the full potential size
    // We won't serialize from this if it's too big for the CoAP buffer.
    JsonDocument doc;
    
    doc["type"] = "device_info";
    doc["version"] = API_VERSION;
    doc["uuid"] = deviceUUID;
    doc["name"] = deviceName;
    doc["deviceType"] = deviceType;
    doc["firmwareVersion"] = firmwareVersion;

    JsonObject capabilities = doc["capabilities"].to<JsonObject>();
    JsonArray actionsArray = capabilities["actions"].to<JsonArray>();
    for (const auto& actionPair : actions) {
        const Action& action = actionPair.second;
        JsonObject actionObj = actionsArray.add<JsonObject>();
        actionObj["id"] = action.id;
        actionObj["name"] = action.name;
        
        // Re-enable parameters
        JsonArray parametersArray = actionObj["parameters"].to<JsonArray>();
        for (const auto& param : action.parameters) {
            JsonObject paramObj = parametersArray.add<JsonObject>();
            paramObj["name"] = param.name;
            paramObj["type"] = param.type;
            if (param.unit.length() > 0) paramObj["unit"] = param.unit;
            if (param.type == "number") {
                paramObj["defaultValue"] = param.defaultValue;
                paramObj["min"] = param.min;
                paramObj["max"] = param.max;
            } else if (param.type == "string") {
                if (param.defaultString.length() > 0) paramObj["defaultValue"] = param.defaultString;
                if (param.min > 0) paramObj["minLength"] = (int)param.min;
                if (param.max > 0) paramObj["maxLength"] = (int)param.max;
            }
            if (param.label.length() > 0) paramObj["label"] = param.label;
            if (param.description.length() > 0) paramObj["description"] = param.description;
        }
    }

    JsonArray eventsArray = capabilities["events"].to<JsonArray>();
    for (const auto& eventPair : events) {
        const Event& event = eventPair.second;
        JsonObject eventObj = eventsArray.add<JsonObject>();
        eventObj["id"] = event.id;
        eventObj["name"] = event.name;
        // Parse the serialized schema back into a JsonObject
        if (event.dataSchemaJson.length() > 0) {
            JsonDocument schemaDoc;
            deserializeJson(schemaDoc, event.dataSchemaJson);
            eventObj["dataSchema"] = schemaDoc.as<JsonObject>();
        }
    }

    JsonObject status = doc["status"].to<JsonObject>();
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
        JsonDocument truncatedDoc; // Small buffer for fallback
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

// Returns CoRE Link Format (RFC 6690) for /.well-known/core discovery
String DeviceHub::getWellKnownCore() {
    String links = "";

    // Device info resource
    links += "</info>;rt=\"device\";ct=50";

    // Action resource
    links += ",</action>;rt=\"action\";ct=50";

    // Legacy v2/actions for compatibility
    links += ",</v2/actions>;rt=\"action\";ct=50";

    // Event resource (if events are registered)
    if (!events.empty()) {
        links += ",</event>;rt=\"event\";obs";
    }

    return links;
}

void DeviceHub::sendDeviceInfo() {
    log("Starting sendDeviceInfo process...");
    String deviceInfo = getDeviceInfo(); // Gets full or truncated info based on size check

    if (!coap) {
        log("Error: CoAP instance is null in sendDeviceInfo.");
        return;
    }

    log("Sending device info via CoAP multicast (NON-CONFIRMABLE - Payload size: %d)...", deviceInfo.length());

    // Send device info to CoAP multicast group (224.0.1.187)
    // This allows the hub daemon to receive periodic updates
    uint16_t messageId = coap->send(getCoapMulticastIP(), COAP_PORT, "device/info",
                                    COAP_NONCON,
                                    COAP_POST, NULL, 0,
                                    (uint8_t*)deviceInfo.c_str(), deviceInfo.length(),
                                    COAP_APPLICATION_JSON);

    if (messageId == 0) {
        log("Error: CoAP send function returned 0 (failed to send).");
    } else {
        log("CoAP multicast send successful. Message ID: %d", messageId);
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
    JsonDocument doc;
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
    uint16_t messageId = coap->send(getBroadcastIP(), COAP_PORT, "message", 
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
    prefsPtr->begin("device-hub", false);
    prefsPtr->putString(key, value);
    prefsPtr->end();
}

String DeviceHub::loadPersistentData(const char* key, const String& defaultValue) {
    prefsPtr->begin("device-hub", false);
    String value = prefsPtr->getString(key, defaultValue);
    prefsPtr->end();
    return value;
}

void DeviceHub::registerEvent(const String& eventId, const String& eventName, const JsonObject& schema) {
    Event event;
    event.id = eventId;
    event.name = eventName;
    // Serialize the schema to a string to avoid dangling JsonObject reference
    // (JsonObject doesn't own memory - the original document might go out of scope)
    if (!schema.isNull()) {
        serializeJson(schema, event.dataSchemaJson);
    } else {
        event.dataSchemaJson = "{}";
    }
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
        prefsPtr->begin("devicehub", false);
        prefsPtr->putString("device_uuid", deviceUUID);
        prefsPtr->end();
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
    prefsPtr->begin("devicehub", false);
    deviceUUID = prefsPtr->getString("device_uuid", "");
    prefsPtr->end();
    
    // If still not valid, generate a new UUID
    if (!isUUIDValid()) {
        generateAndStoreUUID();
    }
}

void DeviceHub::generateAndStoreUUID() {
    // Generate version 4 UUID (random)
    uint8_t uuidBytes[16];
    
    // Get random bytes
    for (int i = 0; i < 16; i += 4) {
        uint32_t randomValue = esp_random();
        uuidBytes[i] = (randomValue >> 24) & 0xFF;
        uuidBytes[i + 1] = (randomValue >> 16) & 0xFF;
        uuidBytes[i + 2] = (randomValue >> 8) & 0xFF;
        uuidBytes[i + 3] = randomValue & 0xFF;
    }
    
    // Set version bits (4 in upper nibble of byte 6)
    uuidBytes[6] = (uuidBytes[6] & 0x0F) | 0x40;
    
    // Set variant bits (10 in upper 2 bits of byte 8)
    uuidBytes[8] = (uuidBytes[8] & 0x3F) | 0x80;
    
    // Format as UUID string (8-4-4-4-12)
    char uuidStr[37];
    snprintf(uuidStr, sizeof(uuidStr),
        "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
        uuidBytes[0], uuidBytes[1], uuidBytes[2], uuidBytes[3],
        uuidBytes[4], uuidBytes[5],
        uuidBytes[6], uuidBytes[7],
        uuidBytes[8], uuidBytes[9],
        uuidBytes[10], uuidBytes[11], uuidBytes[12], uuidBytes[13], uuidBytes[14], uuidBytes[15]
    );
    
    deviceUUID = String(uuidStr);
    
    // Store in persistent storage
    prefsPtr->begin("devicehub", false);
    prefsPtr->putString("device_uuid", deviceUUID);
    prefsPtr->end();
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

WiFiClass DeviceHub::getWiFi() {
    return WiFi;
}

void DeviceHub::handleEmergencyPacket() {
    // Don't try to parse packets if UDP socket hasn't been initialized yet
    // (TCP/IP stack isn't ready until after WiFi.begin() completes)
    if (!emergencyUdpInitialized) {
        return;
    }

    int packetSize = emergencyUdp.parsePacket();
    if (packetSize) {
        char incomingPacket[255];
        int len = emergencyUdp.read(incomingPacket, 255);
        if (len > 0) {
            incomingPacket[len] = 0;
        }
        Serial.printf("Emergency UDP packet received from %s:%d - %s\n", emergencyUdp.remoteIP().toString().c_str(), emergencyUdp.remotePort(), incomingPacket);

        JsonDocument doc;
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
    
    JsonDocument doc;
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

// New public methods for AP fallback functionality

bool DeviceHub::isAPMode() const {
    return isDualCore ? cachedIsAPMode : (currentWiFiMode == WiFiMode::AccessPoint);
}

bool DeviceHub::isConnected() const {
    if (isDualCore) {
        return cachedConnected;
    } else {
        if (currentWiFiMode == WiFiMode::Station) {
            return WiFi.status() == WL_CONNECTED;
        } else if (currentWiFiMode == WiFiMode::AccessPoint) {
            return WiFi.softAPgetStationNum() >= 0;
        }
        return false;
    }
}

String DeviceHub::getAPSSID() const {
    return apSSID;
}

String DeviceHub::getMDNSHostname() const {
    return mdnsHostname;
}

IPAddress DeviceHub::getIP() const {
    if (isDualCore) {
        return cachedIP;
    } else {
        if (currentWiFiMode == WiFiMode::Station) {
            return WiFi.localIP();
        } else if (currentWiFiMode == WiFiMode::AccessPoint) {
            return WiFi.softAPIP();
        }
        return IPAddress(0, 0, 0, 0);
    }
}

void DeviceHub::checkConnection() {
    if (isDualCore) {
        // Send command to network task
        sendNetworkCommand(NetworkCommand::CHECK_CONNECTION);
    } else {
        checkAndReconnect();
    }
}

void DeviceHub::forceAPMode() {
    if (isDualCore) {
        // Send command to network task
        sendNetworkCommand(NetworkCommand::FORCE_AP_MODE);
    } else {
        log("Forcing AP mode...");
        apModeForced = true;
        if (currentWiFiMode == WiFiMode::Station) {
            WiFi.disconnect();
        }
        setupAccessPoint();
        setupMDNS();
    }
}

WiFiMode DeviceHub::getWiFiMode() const {
    return isDualCore ? cachedWiFiMode : currentWiFiMode;
}

// New private methods for AP functionality

bool DeviceHub::attemptWiFiConnection() {
    // Always use multi-WiFi connection since that's the only mode now
    if (!configuredNetworks.empty()) {
        return attemptMultiWiFiConnection();
    }

    log("No WiFi networks configured!");
    currentWiFiMode = WiFiMode::Offline;
    currentSignalStrength = -100;
    return false;
}

void DeviceHub::setupAccessPoint() {
    log("Setting up WiFi Access Point...");
    
    // Ensure clean WiFi state
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(500);  // Give time for WiFi to fully stop
    
    log("Setting WiFi mode to AP...");
    WiFi.mode(WIFI_AP);
    delay(100);
    
    // Configure AP with custom IP
    IPAddress local_ip(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    
    log("Configuring AP IP settings...");
    if (!WiFi.softAPConfig(local_ip, gateway, subnet)) {
        log("ERROR: AP IP config failed!");
        currentWiFiMode = WiFiMode::Offline;
        return;
    }
    
    // Start the access point (with or without password)
    log("Starting AP with SSID: '%s'", apSSID.c_str());
    bool apStarted;
    if (openAPMode) {
        // Open network (no password)
        log("Creating OPEN (passwordless) Access Point...");
        apStarted = WiFi.softAP(apSSID.c_str());
    } else {
        // Secured network (with password)
        log("Creating SECURED Access Point with password...");
        apStarted = WiFi.softAP(apSSID.c_str(), apPassword);
    }
    
    if (apStarted) {
        // Give AP time to fully initialize
        delay(1000);
        
        // Verify AP is actually running
        if (WiFi.getMode() & WIFI_AP) {
            currentWiFiMode = WiFiMode::AccessPoint;
            log("✅ Access Point started successfully!");
            log("SSID: %s", apSSID.c_str());
            log("WiFi Mode: %d (should include AP bit)", WiFi.getMode());
            
            if (openAPMode) {
                log("Security: OPEN (no password required)");
                log("⚠️  Warning: Open network - anyone can connect!");
            } else {
                log("Security: WPA2 with password");
                log("Password: %s", apPassword);
            }
            
            log("IP address: %s", WiFi.softAPIP().toString().c_str());
            log("MAC address: %s", WiFi.softAPmacAddress().c_str());
            
            // Check channel
            int channel = WiFi.channel();
            log("Operating on channel: %d", channel);
            
            if (openAPMode) {
                log("📱 Connect to '%s' network (no password needed)", apSSID.c_str());
                log("🌐 Then access: http://%s.local or http://%s", 
                    mdnsHostname.c_str(), WiFi.softAPIP().toString().c_str());
            }
            
            // Skip AP visibility check - scanning interferes with AP operation on ESP32

            // Initialize UDP sockets now that TCP/IP stack is ready
            initializeUdpSockets();

        } else {
            log("ERROR: AP started but WiFi mode incorrect!");
            currentWiFiMode = WiFiMode::Offline;
        }
    } else {
        log("ERROR: Failed to start Access Point!");
        log("WiFi Mode after failure: %d", WiFi.getMode());
        currentWiFiMode = WiFiMode::Offline;
    }
}

void DeviceHub::setupStationMode() {
    log("Setting up WiFi Station mode...");
    currentWiFiMode = WiFiMode::Station;
    // WiFi is already connected from attemptWiFiConnection()

    // Initialize UDP sockets now that TCP/IP stack is ready
    initializeUdpSockets();
}

void DeviceHub::initializeUdpSockets() {
    // Initialize emergency UDP socket (port 8890)
    // This must be called AFTER WiFi.begin() initializes the TCP/IP stack
    if (!emergencyUdpInitialized) {
        if (emergencyUdp.begin(EMERGENCY_PORT)) {
            log("Emergency UDP: Bound to port %d", EMERGENCY_PORT);
            emergencyUdpInitialized = true;
        } else {
            log("Emergency UDP: Failed to bind to port %d", EMERGENCY_PORT);
        }
    }
}

void DeviceHub::setupMDNS() {
    if (mdnsRunning) {
        MDNS.end(); // Stop existing mDNS
        mdnsRunning = false;
    }
    
    if (currentWiFiMode == WiFiMode::Offline) {
        log("Cannot start mDNS - no network connection");
        return;
    }
    
    log("Starting mDNS responder...");
    if (MDNS.begin(mdnsHostname.c_str())) {
        mdnsRunning = true;
        log("mDNS responder started: http://%s.local", mdnsHostname.c_str());
        
        // Add services
        MDNS.addService("http", "tcp", 80);
        MDNS.addService("ws", "tcp", 80);
        MDNS.addService("coap", "udp", COAP_PORT);
        MDNS.addService("ota", "tcp", 3232);
        
        // Add TXT records with device info
        MDNS.addServiceTxt("http", "tcp", "device", deviceType);
        MDNS.addServiceTxt("http", "tcp", "name", deviceName);
        MDNS.addServiceTxt("http", "tcp", "mode", (currentWiFiMode == WiFiMode::Station) ? "sta" : "ap");
        MDNS.addServiceTxt("http", "tcp", "version", DEVICEHUB_VERSION_2);
        
        log("mDNS services and TXT records added");
    } else {
        log("Failed to start mDNS responder");
    }
}

void DeviceHub::checkAndReconnect() {
    if (apModeForced || currentWiFiMode == WiFiMode::AccessPoint) {
        // In AP mode, check if we should try to reconnect to WiFi
        if (!apModeForced && enableAPFallback) {
            // Attempt to reconnect to WiFi periodically
            static unsigned long lastReconnectAttempt = 0;
            if (millis() - lastReconnectAttempt > 300000) { // Try every 5 minutes
                lastReconnectAttempt = millis();
                log("Periodic attempt to reconnect to WiFi from AP mode...");
                totalReconnectionAttempts++;
                if (attemptWiFiConnection()) {
                    successfulReconnections++;
                    log("Successfully reconnected to WiFi, switching from AP mode");
                    WiFi.softAPdisconnect();
                    setupStationMode();
                    setupMDNS();
                }
            }
        }
        
        // Log AP mode statistics
        if (currentWiFiMode == WiFiMode::AccessPoint) {
            int connectedClients = WiFi.softAPgetStationNum();
            log("AP mode status: %d connected clients", connectedClients);
        }
        
    } else if (currentWiFiMode == WiFiMode::Station) {
        // In station mode, comprehensive connection check
        if (WiFi.status() != WL_CONNECTED) {
            log("Periodic check: WiFi connection lost (Status: %d)", WiFi.status());
            totalReconnectionAttempts++;
            if (attemptWiFiConnection()) {
                successfulReconnections++;
                log("Periodic reconnection successful!");
            } else if (enableAPFallback) {
                log("Periodic reconnection failed, switching to AP mode");
                setupAccessPoint();
                setupMDNS();
            }
        } else {
            // Connection is good, update signal strength
            currentSignalStrength = WiFi.RSSI();
            log("Periodic check: WiFi OK - IP: %s, Signal: %d dBm (%s)",
                WiFi.localIP().toString().c_str(),
                currentSignalStrength,
                getSignalQualityDescription(currentSignalStrength).c_str());

            // Check if we should switch to production network
            checkForProductionNetwork();
        }
    }
    
    // Log connection statistics
    if (totalReconnectionAttempts > 0) {
        float successRate = (float)successfulReconnections / totalReconnectionAttempts * 100.0;
        log("Connection stats: %lu attempts, %lu successful (%.1f%% success rate)",
            totalReconnectionAttempts, successfulReconnections, successRate);
    }
}

String DeviceHub::generateAPSSID() const {
    String ssid = toMdnsHost(String(deviceName));
    // Ensure SSID is valid (max 32 chars, no special chars)
    if (ssid.length() > 32) {
        ssid = ssid.substring(0, 32);
    }
    // Replace any remaining problematic characters
    ssid.replace("-", "");
    ssid.replace("_", "");
    if (ssid.length() == 0) {
        ssid = "DeviceHub";
    }
    return ssid;
}

String DeviceHub::generateMDNSHostname() const {
    return toMdnsHost(String(deviceName));
}

// Enhanced connection monitoring methods

int DeviceHub::getSignalStrength() const {
    if (isDualCore) {
        return cachedSignalStrength;
    } else {
        if (currentWiFiMode == WiFiMode::Station && WiFi.status() == WL_CONNECTED) {
            return WiFi.RSSI();
        }
        return currentSignalStrength;
    }
}

unsigned long DeviceHub::getLastConnectionCheck() const {
    return lastConnectionCheck;
}

bool DeviceHub::isSignalWeak() const {
    return (currentSignalStrength < minSignalStrength);
}

void DeviceHub::setConnectionCheckInterval(int intervalMs) {
    if (intervalMs >= 5000) {  // Minimum 5 seconds
        connectionCheckIntervalMs = intervalMs;
        log("Connection check interval set to %d ms", intervalMs);
    } else {
        log("Warning: Connection check interval must be at least 5000ms");
    }
}

void DeviceHub::setMinSignalStrength(int minStrength) {
    if (minStrength >= -100 && minStrength <= -30) {
        minSignalStrength = minStrength;
        log("Minimum signal strength threshold set to %d dBm", minStrength);
    } else {
        log("Warning: Signal strength must be between -100 and -30 dBm");
    }
}

// New private methods for enhanced monitoring

void DeviceHub::monitorSignalStrength() {
    if (currentWiFiMode != WiFiMode::Station || WiFi.status() != WL_CONNECTED) {
        return;
    }
    
    int newSignalStrength = WiFi.RSSI();
    int signalChange = newSignalStrength - currentSignalStrength;
    currentSignalStrength = newSignalStrength;
    
    log("Signal monitoring: %d dBm (%s, %s%d dBm)", 
        currentSignalStrength,
        getSignalQualityDescription(currentSignalStrength).c_str(),
        signalChange >= 0 ? "+" : "",
        signalChange);
    
    // Check if signal is critically weak
    if (currentSignalStrength < minSignalStrength) {
        log("Warning: WiFi signal is weak (%d dBm < %d dBm threshold)", 
            currentSignalStrength, minSignalStrength);
        
        // If signal is very weak, attempt reconnection
        if (currentSignalStrength < (minSignalStrength - 10)) {
            log("Signal critically weak, attempting reconnection...");
            totalReconnectionAttempts++;
            if (attemptWiFiConnection()) {
                successfulReconnections++;
                log("Reconnection due to weak signal successful!");
            } else if (enableAPFallback) {
                log("Reconnection failed, switching to AP mode due to weak signal");
                setupAccessPoint();
                setupMDNS();
            }
        }
    }
}

String DeviceHub::getSignalQualityDescription(int rssi) const {
    if (rssi >= -30) return "Excellent";
    else if (rssi >= -50) return "Very Good";
    else if (rssi >= -60) return "Good";
    else if (rssi >= -70) return "Fair";
    else if (rssi >= -80) return "Weak";
    else return "Very Weak";
}

// Dual-core detection and setup
bool DeviceHub::detectAndSetupDualCore() {
#if defined(ESP32)
    // Detect core count
    detectedCores = ESP.getChipCores();
    log("Detected %d CPU cores", detectedCores);
    
    // Check if dual-core is available and not forced to single-core
    if (detectedCores >= 2 && !forceSingleCore) {
        log("Dual-core ESP32 detected, initializing FreeRTOS objects...");
        
        if (initializeFreeRTOSObjects()) {
            isDualCore = true;
            log("Dual-core setup successful");
            return true;
        } else {
            log("Dual-core setup failed, falling back to single-core mode");
            cleanupFreeRTOSObjects();
            isDualCore = false;
            return false;
        }
    } else {
        if (forceSingleCore) {
            log("Dual-core available but forced to single-core mode");
        } else {
            log("Single-core device detected");
        }
        isDualCore = false;
        return false;
    }
#else
    log("Non-ESP32 platform detected, using single-core mode");
    detectedCores = 1;
    isDualCore = false;
    return false;
#endif
}

bool DeviceHub::initializeFreeRTOSObjects() {
#if defined(ESP32)
    // Create command queue
    commandQueue = xQueueCreate(DEVICEHUB_QUEUE_SIZE, sizeof(NetworkCommand));
    if (!commandQueue) {
        log("Failed to create command queue");
        return false;
    }
    
    // Create status queue with length 1 for xQueueOverwrite - CRITICAL FIX
    statusQueue = xQueueCreate(1, sizeof(NetworkStatus));
    if (!statusQueue) {
        log("Failed to create status queue");
        return false;
    }
    
    // Create status mutex
    statusMutex = xSemaphoreCreateMutex();
    if (!statusMutex) {
        log("Failed to create status mutex");
        return false;
    }
    
    log("FreeRTOS objects created successfully");
    return true;
#else
    return false;
#endif
}

void DeviceHub::cleanupFreeRTOSObjects() {
#if defined(ESP32)
    if (commandQueue) {
        vQueueDelete(commandQueue);
        commandQueue = nullptr;
    }
    
    if (statusQueue) {
        vQueueDelete(statusQueue);
        statusQueue = nullptr;
    }
    
    if (statusMutex) {
        vSemaphoreDelete(statusMutex);
        statusMutex = nullptr;
    }
    
    log("FreeRTOS objects cleaned up");
#endif
}

void DeviceHub::startNetworkTask() {
#if defined(ESP32)
    if (!isDualCore || networkTaskRunning) {
        return;
    }
    
    log("Starting network task on Core %d...", DEVICEHUB_NETWORK_CORE);
    
    BaseType_t result = xTaskCreatePinnedToCore(
        networkTaskFunction,              // Task function
        "DeviceHub_Network",              // Task name
        DEVICEHUB_NETWORK_TASK_STACK_SIZE, // Stack size
        this,                             // Task parameter
        DEVICEHUB_NETWORK_TASK_PRIORITY,  // Priority
        &networkTaskHandle,               // Task handle
        DEVICEHUB_NETWORK_CORE            // Core ID
    );
    
    if (result == pdPASS) {
        networkTaskRunning = true;
        lastNetworkTaskHealthCheck = millis();
        log("Network task started successfully");
    } else {
        log("Failed to start network task (Error: %d)", result);
        networkTaskHandle = nullptr;
        networkTaskRunning = false;
    }
#endif
}

void DeviceHub::stopNetworkTask() {
#if defined(ESP32)
    if (!networkTaskRunning || !networkTaskHandle) {
        return;
    }
    
    log("Stopping network task...");
    
    // Send stop command to task
    NetworkCommand stopCommand;
    stopCommand.type = NetworkCommand::CHECK_CONNECTION; // Use as stop signal
    strcpy(stopCommand.data, "STOP");
    
    if (commandQueue) {
        xQueueSend(commandQueue, &stopCommand, portMAX_DELAY);
    }
    
    // Give task time to stop gracefully
    delay(100);
    
    // Force delete if still running
    if (networkTaskHandle) {
        vTaskDelete(networkTaskHandle);
        networkTaskHandle = nullptr;
    }
    
    networkTaskRunning = false;
    log("Network task stopped");
#endif
}

void DeviceHub::networkTaskFunction(void* parameter) {
#if defined(ESP32)
    DeviceHub* deviceHub = static_cast<DeviceHub*>(parameter);
    deviceHub->handleNetworkOperations();
#endif
}

void DeviceHub::handleNetworkOperations() {
#if defined(ESP32)
    // REMOVE THE ENTIRE LOW-LEVEL ESP-IDF WIFI INITIALIZATION BLOCK
    // The global initialization is now done in DeviceHub::begin()
    // The Arduino WiFi API calls below (attemptWiFiConnection, setupAccessPoint)
    // will handle the creation of STA/AP netif instances correctly.
#endif
    log("Network task started on Core %d", xPortGetCoreID());
    
    // Initialize WiFi on network core using Arduino WiFi API methods
    if (apModeForced) {
        log("Network Task: AP mode forced, setting up Access Point...");
        setupAccessPoint();
    } else if (attemptWiFiConnection()) {
        log("Network Task: WiFi connection successful, setting up Station mode...");
        setupStationMode();
    } else if (enableAPFallback) {
        log("Network Task: WiFi connection failed, falling back to Access Point mode...");
        setupAccessPoint();
    } else {
        log("Network Task: WiFi connection failed and AP fallback disabled. Operating offline.");
        safeUpdateWiFiMode(WiFiMode::Offline);
    }
    
    // Setup mDNS if we have a network connection
    if (currentWiFiMode != WiFiMode::Offline) {
        setupMDNS();
    }

    // Start OTA helper in dual-core mode after WiFi connects
    if (currentWiFiMode == WiFiMode::Station && !configuredNetworks.empty()) {
        log("Network Task: Starting OTA helper...");
        for (const auto& network : configuredNetworks) {
            if (network.ssid == connectedSSID) {
                otaHelper.start(network.ssid.c_str(), network.password.c_str(), deviceName, network.password.c_str(), 3232, 115200);
                break;
            }
        }
    }

    // Update initial status
    NetworkStatus status;
    memset(&status, 0, sizeof(status));  // Zero-initialize all fields including char arrays
    status.wifiMode = currentWiFiMode;
    status.connected = (currentWiFiMode == WiFiMode::Station) ? (WiFi.status() == WL_CONNECTED) : true;
    status.signalStrength = currentSignalStrength;
    status.currentIP = getIP();
    status.lastUpdate = millis();
    status.isAPMode = (currentWiFiMode == WiFiMode::AccessPoint);
    
    if (statusQueue) {
        xQueueSend(statusQueue, &status, 0);
    }
    
    unsigned long lastConnectionCheck = 0;
    unsigned long lastSignalCheck = 0;
    unsigned long lastHeartbeat = 0;
    
    // Main network task loop
    while (true) {
        unsigned long currentTime = millis();
        
        // Check for commands from main core
        NetworkCommand command;
        if (commandQueue && xQueueReceive(commandQueue, &command, 10) == pdTRUE) {
            if (strcmp(command.data, "STOP") == 0) {
                log("Network task received stop command");
                break;
            }
            
            // Handle other commands
            switch (command.type) {
                case NetworkCommand::WIFI_RECONNECT:
                    totalReconnectionAttempts++;
                    if (attemptWiFiConnection()) {
                        successfulReconnections++;
                    }
                    break;
                case NetworkCommand::FORCE_AP_MODE:
                    setupAccessPoint();
                    setupMDNS();
                    break;
                case NetworkCommand::CHECK_CONNECTION:
                    checkAndReconnect();
                    break;
                default:
                    break;
            }
        }
        
        // Periodic WiFi monitoring
        if (currentTime - lastConnectionCheck > connectionCheckIntervalMs) {
            lastConnectionCheck = currentTime;
            checkAndReconnect();
        }
        
        // Signal strength monitoring
        if (currentWiFiMode == WiFiMode::Station && 
            currentTime - lastSignalCheck > DEFAULT_SIGNAL_STRENGTH_CHECK_INTERVAL_MS) {
            lastSignalCheck = currentTime;
            monitorSignalStrength();
        }
        
        // Send periodic heartbeat/device info
        if (currentTime - lastHeartbeat > 15000) { // Every 15 seconds
            lastHeartbeat = currentTime;
            sendPeriodicUpdate();
        }
        
        // Update status for main core
        status.wifiMode = currentWiFiMode;
        status.connected = (currentWiFiMode == WiFiMode::Station) ? (WiFi.status() == WL_CONNECTED) : true;
        status.signalStrength = currentSignalStrength;
        status.currentIP = getIP();
        status.lastUpdate = currentTime;
        status.isAPMode = (currentWiFiMode == WiFiMode::AccessPoint);
        
        if (statusQueue) {
            xQueueOverwrite(statusQueue, &status); // Overwrite to keep latest status
        }
        
        // Small delay to prevent task starvation
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    log("Network task ending");
    networkTaskRunning = false;
    vTaskDelete(nullptr);
}

// New dual-core public API methods

bool DeviceHub::isDualCoreMode() const {
    return isDualCore;
}

bool DeviceHub::isSingleCoreMode() const {
    return !isDualCore;
}

int DeviceHub::getCoreCount() const {
    return detectedCores;
}

bool DeviceHub::isNetworkTaskRunning() const {
    return networkTaskRunning;
}

// Dual-core helper methods

bool DeviceHub::sendNetworkCommand(NetworkCommand::Type type, const String& data, const JsonDocument& payload) {
#if defined(ESP32)
    if (!isDualCore || !commandQueue) {
        return false;
    }

    NetworkCommand command;
    command.type = type;
    // Copy String to fixed char array (truncate if needed)
    strncpy(command.data, data.c_str(), sizeof(command.data) - 1);
    command.data[sizeof(command.data) - 1] = '\0';
    // Note: payload parameter is ignored - JsonDocument cannot be safely passed through FreeRTOS queues

    return xQueueSend(commandQueue, &command, pdMS_TO_TICKS(100)) == pdTRUE;
#else
    return false;
#endif
}

void DeviceHub::updateStatusFromNetworkTask(const NetworkStatus& status) {
    if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        cachedWiFiMode = status.wifiMode;
        cachedConnected = status.connected;
        cachedSignalStrength = status.signalStrength;
        cachedIP = status.currentIP;
        cachedIsAPMode = status.isAPMode;
        
        currentNetworkStatus = status;
        
        xSemaphoreGive(statusMutex);
    }
}

void DeviceHub::checkNetworkTaskHealth() {
#if defined(ESP32)
    if (!isDualCore || !networkTaskRunning) {
        return;
    }
    
    unsigned long currentTime = millis();
    
    // Check if we've received a status update recently
    if (currentTime - currentNetworkStatus.lastUpdate > 30000) { // 30 seconds without update
        log("Warning: Network task appears unresponsive (last update %lu ms ago)", 
            currentTime - currentNetworkStatus.lastUpdate);
        
        // Try to restart the network task
        if (currentTime - lastNetworkTaskHealthCheck > 60000) { // Only restart once per minute
            lastNetworkTaskHealthCheck = currentTime;
            log("Attempting to restart network task...");
            
            stopNetworkTask();
            delay(500);
            startNetworkTask();
        }
    }
#endif
}

// Thread-safe wrapper methods

void DeviceHub::safeUpdateWiFiMode(WiFiMode mode) {
    if (isDualCore && statusMutex) {
        if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            currentWiFiMode = mode;
            cachedWiFiMode = mode;
            xSemaphoreGive(statusMutex);
        }
    } else {
        currentWiFiMode = mode;
    }
}

void DeviceHub::safeUpdateConnectionStatus(bool connected) {
    if (isDualCore && statusMutex) {
        if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            cachedConnected = connected;
            xSemaphoreGive(statusMutex);
        }
    }
}

void DeviceHub::safeUpdateSignalStrength(int strength) {
    if (isDualCore && statusMutex) {
        if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            currentSignalStrength = strength;
            cachedSignalStrength = strength;
            xSemaphoreGive(statusMutex);
        }
    } else {
        currentSignalStrength = strength;
    }
}

void DeviceHub::safeUpdateIP(IPAddress ip) {
    if (isDualCore && statusMutex) {
        if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            cachedIP = ip;
            xSemaphoreGive(statusMutex);
        }
    }
}

bool DeviceHub::isAPOpen() const {
    return openAPMode;
}

bool DeviceHub::isAPOnlyMode() const {
    return apModeForced && configuredNetworks.empty();
}

void DeviceHub::initializeNVS() {
    // Initialize NVS first - CRITICAL for Preferences to work
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        log("NVS partition was truncated, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        log("Error initializing NVS: %s", esp_err_to_name(err));
    } else {
        log("NVS initialized successfully");
    }
}

// v2-only emergency: send as a CoAP device_event with event='emergency'
void DeviceHub::sendEmergency(const String& message) {
    JsonDocument doc;
    doc["type"] = "device_event";
    doc["version"] = DEVICEHUB_VERSION_2;
    doc["uuid"] = deviceUUID;
    doc["event"] = "emergency";
    doc["data"]["message"] = message;

    String payload;
    serializeJson(doc, payload);
    // NON-confirmable is fine for alarms; hub listens on 5683 and joins 224.0.1.187
    sendCoAPMessage(payload, COAP_NONCON);
}

// =============================================
// Multi-WiFi Support Methods
// =============================================

void DeviceHub::addNetwork(const String& ssid, const String& password,
                           WiFiEnvironment env, int priority) {
    WiFiNetwork network;
    network.ssid = ssid;
    network.password = password;
    network.environment = env;
    network.priority = priority;
    configuredNetworks.push_back(network);
    log("Added network: %s (env: %s, priority: %d)",
        ssid.c_str(),
        env == WiFiEnvironment::Production ? "production" :
        env == WiFiEnvironment::Development ? "development" : "unknown",
        priority);
}

void DeviceHub::addNetwork(const String& ssid, const String& password,
                           const String& envStr, int priority) {
    addNetwork(ssid, password, parseEnvironment(envStr), priority);
}

WiFiEnvironment DeviceHub::parseEnvironment(const String& envStr) {
    String lower = envStr;
    lower.toLowerCase();
    if (lower == "production" || lower == "prod" || lower == "p") {
        return WiFiEnvironment::Production;
    } else if (lower == "development" || lower == "dev" || lower == "d") {
        return WiFiEnvironment::Development;
    }
    return WiFiEnvironment::Unknown;
}

void DeviceHub::autoConfigureNetworks() {
    log("Auto-configuring WiFi networks from build defines...");

#ifdef WIFI_MULTI_ENABLED
    // Multi-network mode - add all configured networks

    #ifdef WIFI_1_SSID
        #ifdef WIFI_1_ENV
            addNetwork(WIFI_1_SSID, WIFI_1_PASSWORD, WIFI_1_ENV, 1);
        #else
            addNetwork(WIFI_1_SSID, WIFI_1_PASSWORD, WiFiEnvironment::Production, 1);
        #endif
    #endif

    #ifdef WIFI_2_SSID
        #ifdef WIFI_2_ENV
            addNetwork(WIFI_2_SSID, WIFI_2_PASSWORD, WIFI_2_ENV, 2);
        #else
            addNetwork(WIFI_2_SSID, WIFI_2_PASSWORD, WiFiEnvironment::Development, 2);
        #endif
    #endif

    #ifdef WIFI_3_SSID
        #ifdef WIFI_3_ENV
            addNetwork(WIFI_3_SSID, WIFI_3_PASSWORD, WIFI_3_ENV, 3);
        #else
            addNetwork(WIFI_3_SSID, WIFI_3_PASSWORD, WiFiEnvironment::Unknown, 3);
        #endif
    #endif

    #ifdef WIFI_4_SSID
        #ifdef WIFI_4_ENV
            addNetwork(WIFI_4_SSID, WIFI_4_PASSWORD, WIFI_4_ENV, 4);
        #else
            addNetwork(WIFI_4_SSID, WIFI_4_PASSWORD, WiFiEnvironment::Unknown, 4);
        #endif
    #endif

    #ifdef WIFI_5_SSID
        #ifdef WIFI_5_ENV
            addNetwork(WIFI_5_SSID, WIFI_5_PASSWORD, WIFI_5_ENV, 5);
        #else
            addNetwork(WIFI_5_SSID, WIFI_5_PASSWORD, WiFiEnvironment::Unknown, 5);
        #endif
    #endif

#else
    // Legacy single network mode
    #ifdef WIFI_SSID
        #ifdef WIFI_PASSWORD
            addNetwork(WIFI_SSID, WIFI_PASSWORD, WiFiEnvironment::Production, 1);
        #else
            addNetwork(WIFI_SSID, "", WiFiEnvironment::Production, 1);
        #endif
    #endif
#endif

    log("Configured %d WiFi network(s)", configuredNetworks.size());
}

void DeviceHub::scanAndSortNetworks() {
    log("Scanning for available networks...");

    int n = WiFi.scanNetworks();
    log("Found %d networks", n);

    // Create a map of SSID -> signal strength
    std::map<String, int> availableNetworks;
    for (int i = 0; i < n; i++) {
        String ssidFound = WiFi.SSID(i);
        int rssi = WiFi.RSSI(i);
        if (availableNetworks.find(ssidFound) == availableNetworks.end() ||
            availableNetworks[ssidFound] < rssi) {
            availableNetworks[ssidFound] = rssi;
        }
        log("  %s: %d dBm", ssidFound.c_str(), rssi);
    }

    // Sort networks: available first (by priority), then unavailable
    std::sort(configuredNetworks.begin(), configuredNetworks.end(),
        [&availableNetworks](const WiFiNetwork& a, const WiFiNetwork& b) {
            bool aAvailable = availableNetworks.find(a.ssid) != availableNetworks.end();
            bool bAvailable = availableNetworks.find(b.ssid) != availableNetworks.end();

            // Available networks come first
            if (aAvailable != bAvailable) {
                return aAvailable;
            }

            // Both available or both unavailable - sort by priority
            return a.priority < b.priority;
        });

    WiFi.scanDelete();
}

bool DeviceHub::tryConnectToNetwork(const WiFiNetwork& network, unsigned long timeoutMs) {
    log("Trying to connect to: %s", network.ssid.c_str());

    WiFi.mode(WIFI_OFF);
    delay(100);
    WiFi.mode(WIFI_STA);
    delay(100);

    WiFi.begin(network.ssid.c_str(), network.password.c_str());

    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - startTime) < timeoutMs) {
        delay(250);
        if (debugEnabled && debugSerial) {
            debugSerial->print(".");
        }
    }

    if (WiFi.status() == WL_CONNECTED) {
        if (debugEnabled && debugSerial) {
            debugSerial->println();
        }
        log("Connected to %s!", network.ssid.c_str());
        log("IP: %s, RSSI: %d dBm", WiFi.localIP().toString().c_str(), WiFi.RSSI());

        // Update state
        connectedSSID = network.ssid;
        currentEnvironment = network.environment;
        currentWiFiMode = WiFiMode::Station;
        currentSignalStrength = WiFi.RSSI();

        return true;
    }

    if (debugEnabled && debugSerial) {
        debugSerial->println();
    }
    log("Failed to connect to %s (status: %d)", network.ssid.c_str(), WiFi.status());
    return false;
}

bool DeviceHub::attemptMultiWiFiConnection() {
    if (configuredNetworks.empty()) {
        log("No networks configured for multi-WiFi!");
        return false;
    }

    // Scan and sort networks by availability and priority
    scanAndSortNetworks();

    // Calculate timeout per network
    unsigned long perNetworkTimeout = wifiTimeoutMs / configuredNetworks.size();
    if (perNetworkTimeout < 5000) {
        perNetworkTimeout = 5000;  // Minimum 5 seconds per network
    }

    // Try each network in order
    for (const auto& network : configuredNetworks) {
        if (tryConnectToNetwork(network, perNetworkTimeout)) {
            log("=== Connected to %s ===", network.ssid.c_str());
            log("Environment: %s", getCurrentEnvironmentString().c_str());
            log("IP Address: %s", WiFi.localIP().toString().c_str());
            log("Signal: %d dBm", WiFi.RSSI());
            return true;
        }
    }

    log("Failed to connect to any configured network");
    return false;
}

WiFiEnvironment DeviceHub::getCurrentEnvironment() const {
    return currentEnvironment;
}

String DeviceHub::getCurrentEnvironmentString() const {
    switch (currentEnvironment) {
        case WiFiEnvironment::Production:
            return "production";
        case WiFiEnvironment::Development:
            return "development";
        default:
            return "unknown";
    }
}

bool DeviceHub::isProduction() const {
    return currentEnvironment == WiFiEnvironment::Production;
}

bool DeviceHub::isDevelopment() const {
    return currentEnvironment == WiFiEnvironment::Development;
}

String DeviceHub::getConnectedSSID() const {
    return connectedSSID;
}

const std::vector<WiFiNetwork>& DeviceHub::getConfiguredNetworks() const {
    return configuredNetworks;
}

void DeviceHub::checkForProductionNetwork() {
    // Only check if we're connected to a non-production network
    if (currentEnvironment == WiFiEnvironment::Production ||
        currentWiFiMode != WiFiMode::Station ||
        WiFi.status() != WL_CONNECTED) {
        return;
    }

    // Check if it's time to scan for production network
    unsigned long currentTime = millis();
    if (currentTime - lastProductionCheck < productionCheckIntervalMs) {
        return;
    }
    lastProductionCheck = currentTime;

    log("Checking if production network is available (currently on %s - %s)...",
        connectedSSID.c_str(), getCurrentEnvironmentString().c_str());

    // Find the production network(s) in our config
    WiFiNetwork* productionNetwork = nullptr;
    for (auto& network : configuredNetworks) {
        if (network.environment == WiFiEnvironment::Production) {
            productionNetwork = &network;
            break;  // Use the first (highest priority) production network
        }
    }

    if (!productionNetwork) {
        log("No production network configured");
        return;
    }

    // Quick scan to see if production network is available
    log("Scanning for production network: %s", productionNetwork->ssid.c_str());
    int n = WiFi.scanNetworks(false, false, false, 300);  // Quick scan, 300ms per channel

    bool productionAvailable = false;
    int productionRssi = -100;
    for (int i = 0; i < n; i++) {
        if (WiFi.SSID(i) == productionNetwork->ssid) {
            productionAvailable = true;
            productionRssi = WiFi.RSSI(i);
            break;
        }
    }
    WiFi.scanDelete();

    if (!productionAvailable) {
        log("Production network '%s' not available yet", productionNetwork->ssid.c_str());
        return;
    }

    log("Production network '%s' is available (signal: %d dBm)! Switching...",
        productionNetwork->ssid.c_str(), productionRssi);

    // Disconnect from current network and connect to production
    WiFi.disconnect(true);
    delay(100);

    if (tryConnectToNetwork(*productionNetwork, 10000)) {
        log("=== Switched to PRODUCTION network: %s ===", productionNetwork->ssid.c_str());
        log("IP Address: %s", WiFi.localIP().toString().c_str());
        log("Signal: %d dBm", WiFi.RSSI());
        setupMDNS();  // Re-setup mDNS with new connection
    } else {
        // Failed to connect to production, try to reconnect to any available network
        log("Failed to switch to production, reconnecting to any available network...");
        if (!attemptMultiWiFiConnection()) {
            log("Failed to reconnect to any network!");
            if (enableAPFallback) {
                setupAccessPoint();
                setupMDNS();
            }
        }
    }
}

// =============================================
// BLE iBeacon Support Methods
// =============================================

bool DeviceHub::startBLEBeacon() {
#if defined(ESP32)
    if (!bleBeaconConfigured) {
        log("BLE beacon not configured. Call setBLEBeacon() before begin().");
        return false;
    }

    if (deviceUUID.length() != 36) {
        log("Error: Device UUID not available. Call startBLEBeacon() after begin().");
        return false;
    }

    if (bleBeaconActive) {
        log("BLE beacon already active");
        return true;
    }

    log("Starting BLE iBeacon (Major=%d, Minor=%d)...", bleBeaconMajor, bleBeaconMinor);

    BLEDevice::init("");
    configureBLEAdvertising();

    bleBeaconActive = true;
    log("BLE iBeacon started successfully");
    return true;
#else
    log("BLE iBeacon not supported on this platform");
    return false;
#endif
}

void DeviceHub::stopBLEBeacon() {
#if defined(ESP32)
    if (!bleBeaconActive) {
        return;
    }

    log("Stopping BLE iBeacon...");

    BLEDevice::getAdvertising()->stop();
    BLEDevice::deinit(true);

    bleBeaconActive = false;
    log("BLE iBeacon stopped, memory released");
#endif
}

bool DeviceHub::isBLEBeaconActive() const {
    return bleBeaconActive;
}

void DeviceHub::configureBLEAdvertising() {
#if defined(ESP32)
    BLEBeacon beacon;
    beacon.setManufacturerId(0x004C);  // Apple's company ID (required for iBeacon)
    beacon.setProximityUUID(BLEUUID::fromString(deviceUUID.c_str()));
    beacon.setMajor(bleBeaconMajor);
    beacon.setMinor(bleBeaconMinor);
    beacon.setSignalPower(bleBeaconTxPower);

    BLEAdvertisementData advData;
    advData.setFlags(0x06);  // BR_EDR_NOT_SUPPORTED | LE_GENERAL_DISC

    std::string strServiceData = "";
    strServiceData += (char)26;    // Length of remaining data
    strServiceData += (char)0xFF;  // Type: Manufacturer Specific Data
    strServiceData += beacon.getData();
    advData.addData(strServiceData);

    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->setAdvertisementData(advData);
    BLEAdvertisementData emptyScanResponse;
    pAdvertising->setScanResponseData(emptyScanResponse);
    pAdvertising->setAdvertisementType(ADV_TYPE_NONCONN_IND);
    pAdvertising->start();

    log("BLE advertising started: UUID=%s, Major=%d, Minor=%d",
        deviceUUID.c_str(), bleBeaconMajor, bleBeaconMinor);
#endif
}

// =============================================
// HTTP OTA Update
// =============================================

bool DeviceHub::performOTAUpdate(const char* url) {
#if defined(ESP32)
    log("OTA: Starting HTTP update from %s", url);

    HTTPClient http;
    http.begin(url);
    int httpCode = http.GET();

    if (httpCode != HTTP_CODE_OK) {
        log("OTA: HTTP GET failed, code: %d", httpCode);
        http.end();
        return false;
    }

    int contentLength = http.getSize();
    if (contentLength <= 0) {
        log("OTA: Invalid content length: %d", contentLength);
        http.end();
        return false;
    }

    log("OTA: Firmware size: %d bytes", contentLength);

    if (!Update.begin(contentLength, U_FLASH)) {
        log("OTA: Update.begin() failed: %s", Update.errorString());
        http.end();
        return false;
    }

    WiFiClient* stream = http.getStreamPtr();
    size_t written = Update.writeStream(*stream);
    http.end();

    if (written != (size_t)contentLength) {
        log("OTA: Write mismatch: wrote %d of %d bytes", written, contentLength);
        Update.abort();
        return false;
    }

    if (!Update.end() || !Update.isFinished()) {
        log("OTA: Update.end() failed: %s", Update.errorString());
        return false;
    }

    log("OTA: Update successful! Rebooting...");
    delay(500);
    ESP.restart();
    return true;  // Never reached after restart
#else
    log("OTA: HTTP OTA not supported on this platform");
    return false;
#endif
}

