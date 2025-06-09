#include "DeviceHub.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include <nvs_flash.h>
#include <esp_err.h>
#include <lwip/tcpip.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_wifi.h>

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
      debugSerial(nullptr), debugEnabled(false),
      apPassword(DEFAULT_AP_PASSWORD), wifiTimeoutMs(DEFAULT_WIFI_TIMEOUT_MS), enableAPFallback(true),
      currentWiFiMode(WiFiMode::Offline), lastConnectionCheck(0), lastSignalCheck(0),
      connectionCheckIntervalMs(DEFAULT_CONNECTION_CHECK_INTERVAL_MS), 
      minSignalStrength(DEFAULT_MIN_SIGNAL_STRENGTH), currentSignalStrength(-100),
      apModeForced(false), totalReconnectionAttempts(0), successfulReconnections(0), openAPMode(DEFAULT_AP_OPEN_NETWORK),
      forceSingleCore(false), isDualCore(false), detectedCores(1),
      networkTaskHandle(nullptr), commandQueue(nullptr), statusQueue(nullptr), statusMutex(nullptr),
      networkTaskRunning(false), lastNetworkTaskHealthCheck(0),
      cachedWiFiMode(WiFiMode::Offline), cachedConnected(false), cachedSignalStrength(-100), cachedIsAPMode(false) {
    initializeNVS();
    prefs.begin("devicehub", false);
    apSSID = generateAPSSID();
    mdnsHostname = generateMDNSHostname();
    currentNetworkStatus.wifiMode = WiFiMode::Offline;
    currentNetworkStatus.connected = false;
    currentNetworkStatus.signalStrength = -100;
    currentNetworkStatus.lastUpdate = 0;
    currentNetworkStatus.isAPMode = false;
}

DeviceHub::DeviceHub(const char* ssid, const char* password, const char* deviceName, const char* deviceType,
                     const char* apPassword, int wifiTimeoutMs, bool enableAPFallback)
    : ssid(ssid), password(password), deviceName(deviceName), deviceType(deviceType), 
      currentState(DeviceState::Normal), dtlsEnabled(false),
      coap(nullptr), secureCoap(nullptr),
      debugSerial(nullptr), debugEnabled(false),
      apPassword(apPassword), wifiTimeoutMs(wifiTimeoutMs), enableAPFallback(enableAPFallback),
      currentWiFiMode(WiFiMode::Offline), lastConnectionCheck(0), lastSignalCheck(0),
      connectionCheckIntervalMs(DEFAULT_CONNECTION_CHECK_INTERVAL_MS),
      minSignalStrength(DEFAULT_MIN_SIGNAL_STRENGTH), currentSignalStrength(-100),
      apModeForced(false), totalReconnectionAttempts(0), successfulReconnections(0), openAPMode(DEFAULT_AP_OPEN_NETWORK),
      forceSingleCore(false), isDualCore(false), detectedCores(1),
      networkTaskHandle(nullptr), commandQueue(nullptr), statusQueue(nullptr), statusMutex(nullptr),
      networkTaskRunning(false), lastNetworkTaskHealthCheck(0),
      cachedWiFiMode(WiFiMode::Offline), cachedConnected(false), cachedSignalStrength(-100), cachedIsAPMode(false) {
    initializeNVS();
    if (strlen(this->apPassword) < 8) { this->apPassword = DEFAULT_AP_PASSWORD; log("Warning: AP password too short, using default: %s", DEFAULT_AP_PASSWORD); }
    prefs.begin("devicehub", false);
    apSSID = generateAPSSID();
    mdnsHostname = generateMDNSHostname();
    currentNetworkStatus.wifiMode = WiFiMode::Offline;
    currentNetworkStatus.connected = false;
    currentNetworkStatus.signalStrength = -100;
    currentNetworkStatus.lastUpdate = 0;
    currentNetworkStatus.isAPMode = false;
}

DeviceHub::DeviceHub(const char* ssid, const char* password, const char* deviceName, const char* deviceType,
                     const char* apPassword, int wifiTimeoutMs, bool enableAPFallback, 
                     int connectionCheckIntervalMs, int minSignalStrength)
    : ssid(ssid), password(password), deviceName(deviceName), deviceType(deviceType), 
      currentState(DeviceState::Normal), dtlsEnabled(false),
      coap(nullptr), secureCoap(nullptr),
      debugSerial(nullptr), debugEnabled(false),
      apPassword(apPassword), wifiTimeoutMs(wifiTimeoutMs), enableAPFallback(enableAPFallback),
      currentWiFiMode(WiFiMode::Offline), lastConnectionCheck(0), lastSignalCheck(0),
      connectionCheckIntervalMs(connectionCheckIntervalMs), minSignalStrength(minSignalStrength),
      currentSignalStrength(-100), apModeForced(false), 
      totalReconnectionAttempts(0), successfulReconnections(0), openAPMode(DEFAULT_AP_OPEN_NETWORK),
      forceSingleCore(false), isDualCore(false), detectedCores(1),
      networkTaskHandle(nullptr), commandQueue(nullptr), statusQueue(nullptr), statusMutex(nullptr),
      networkTaskRunning(false), lastNetworkTaskHealthCheck(0),
      cachedWiFiMode(WiFiMode::Offline), cachedConnected(false), cachedSignalStrength(-100), cachedIsAPMode(false) {
    initializeNVS();
    if (strlen(this->apPassword) < 8) { this->apPassword = DEFAULT_AP_PASSWORD; log("Warning: AP password too short, using default: %s", DEFAULT_AP_PASSWORD); }
    if (this->connectionCheckIntervalMs < 5000) { this->connectionCheckIntervalMs = 5000; log("Warning: Connection check interval too short, using 5000ms"); }
    if (this->minSignalStrength > -30 || this->minSignalStrength < -100) { this->minSignalStrength = DEFAULT_MIN_SIGNAL_STRENGTH; log("Warning: Invalid signal strength threshold, using default: %d dBm", DEFAULT_MIN_SIGNAL_STRENGTH); }
    prefs.begin("devicehub", false);
    apSSID = generateAPSSID();
    mdnsHostname = generateMDNSHostname();
    currentNetworkStatus.wifiMode = WiFiMode::Offline;
    currentNetworkStatus.connected = false;
    currentNetworkStatus.signalStrength = -100;
    currentNetworkStatus.lastUpdate = 0;
    currentNetworkStatus.isAPMode = false;
}

DeviceHub::DeviceHub(const char* ssid, const char* password, const char* deviceName, const char* deviceType,
                     const char* apPassword, int wifiTimeoutMs, bool enableAPFallback, 
                     int connectionCheckIntervalMs, int minSignalStrength, bool forceSingleCore)
    : ssid(ssid), password(password), deviceName(deviceName), deviceType(deviceType), 
      currentState(DeviceState::Normal), dtlsEnabled(false),
      coap(nullptr), secureCoap(nullptr),
      debugSerial(nullptr), debugEnabled(false),
      apPassword(apPassword), wifiTimeoutMs(wifiTimeoutMs), enableAPFallback(enableAPFallback),
      currentWiFiMode(WiFiMode::Offline), lastConnectionCheck(0), lastSignalCheck(0),
      connectionCheckIntervalMs(connectionCheckIntervalMs), minSignalStrength(minSignalStrength),
      currentSignalStrength(-100), apModeForced(false), 
      totalReconnectionAttempts(0), successfulReconnections(0), openAPMode(DEFAULT_AP_OPEN_NETWORK),
      forceSingleCore(forceSingleCore), isDualCore(false), detectedCores(1),
      networkTaskHandle(nullptr), commandQueue(nullptr), statusQueue(nullptr), statusMutex(nullptr),
      networkTaskRunning(false), lastNetworkTaskHealthCheck(0),
      cachedWiFiMode(WiFiMode::Offline), cachedConnected(false), cachedSignalStrength(-100), cachedIsAPMode(false) {
    initializeNVS();
    if (strlen(this->apPassword) < 8) { this->apPassword = DEFAULT_AP_PASSWORD; log("Warning: AP password too short, using default: %s", DEFAULT_AP_PASSWORD); }
    if (this->connectionCheckIntervalMs < 5000) { this->connectionCheckIntervalMs = 5000; log("Warning: Connection check interval too short, using 5000ms"); }
    if (this->minSignalStrength > -30 || this->minSignalStrength < -100) { this->minSignalStrength = DEFAULT_MIN_SIGNAL_STRENGTH; log("Warning: Invalid signal strength threshold, using default: %d dBm", DEFAULT_MIN_SIGNAL_STRENGTH); }
    prefs.begin("devicehub", false);
    apSSID = generateAPSSID();
    mdnsHostname = generateMDNSHostname();
    currentNetworkStatus.wifiMode = WiFiMode::Offline;
    currentNetworkStatus.connected = false;
    currentNetworkStatus.signalStrength = -100;
    currentNetworkStatus.lastUpdate = 0;
    currentNetworkStatus.isAPMode = false;
}

DeviceHub::DeviceHub(const char* ssid, const char* password, const char* deviceName, const char* deviceType,
                     const char* apPassword, int wifiTimeoutMs, bool enableAPFallback, 
                     int connectionCheckIntervalMs, int minSignalStrength, bool forceSingleCore, bool openAP)
    : ssid(ssid), password(password), deviceName(deviceName), deviceType(deviceType), 
      currentState(DeviceState::Normal), dtlsEnabled(false),
      coap(nullptr), secureCoap(nullptr),
      debugSerial(nullptr), debugEnabled(false),
      apPassword(apPassword), wifiTimeoutMs(wifiTimeoutMs), enableAPFallback(enableAPFallback),
      currentWiFiMode(WiFiMode::Offline), lastConnectionCheck(0), lastSignalCheck(0),
      connectionCheckIntervalMs(connectionCheckIntervalMs), minSignalStrength(minSignalStrength),
      currentSignalStrength(-100), apModeForced(false), 
      totalReconnectionAttempts(0), successfulReconnections(0), openAPMode(openAP),
      forceSingleCore(forceSingleCore), isDualCore(false), detectedCores(1),
      networkTaskHandle(nullptr), commandQueue(nullptr), statusQueue(nullptr), statusMutex(nullptr),
      networkTaskRunning(false), lastNetworkTaskHealthCheck(0),
      cachedWiFiMode(WiFiMode::Offline), cachedConnected(false), cachedSignalStrength(-100), cachedIsAPMode(false) {
    initializeNVS();
    if (!this->openAPMode && strlen(this->apPassword) < 8) { this->apPassword = DEFAULT_AP_PASSWORD; this->openAPMode = false; log("Warning: AP password too short for secure mode, using default: %s", DEFAULT_AP_PASSWORD); }
    if (this->connectionCheckIntervalMs < 5000) { this->connectionCheckIntervalMs = 5000; log("Warning: Connection check interval too short, using 5000ms"); }
    if (this->minSignalStrength > -30 || this->minSignalStrength < -100) { this->minSignalStrength = DEFAULT_MIN_SIGNAL_STRENGTH; log("Warning: Invalid signal strength threshold, using default: %d dBm", DEFAULT_MIN_SIGNAL_STRENGTH); }
    prefs.begin("devicehub", false);
    apSSID = generateAPSSID();
    mdnsHostname = generateMDNSHostname();
    currentNetworkStatus.wifiMode = WiFiMode::Offline;
    currentNetworkStatus.connected = false;
    currentNetworkStatus.signalStrength = -100;
    currentNetworkStatus.lastUpdate = 0;
    currentNetworkStatus.isAPMode = false;
    if (this->openAPMode) { log("Open (passwordless) AP mode enabled for easier device setup"); }
}

// AP-only constructor - forces AP mode without any WiFi connection attempts
DeviceHub::DeviceHub(const char* deviceName, const char* deviceType, const char* apPassword, bool openAP)
    : ssid(""), password(""), deviceName(deviceName), deviceType(deviceType), 
      currentState(DeviceState::Normal), dtlsEnabled(false),
      coap(nullptr), secureCoap(nullptr),
      debugSerial(nullptr), debugEnabled(false),
      apPassword(apPassword), wifiTimeoutMs(0), enableAPFallback(false),
      currentWiFiMode(WiFiMode::Offline), lastConnectionCheck(0), lastSignalCheck(0),
      connectionCheckIntervalMs(DEFAULT_CONNECTION_CHECK_INTERVAL_MS), 
      minSignalStrength(DEFAULT_MIN_SIGNAL_STRENGTH), currentSignalStrength(-100),
      apModeForced(true), totalReconnectionAttempts(0), successfulReconnections(0), openAPMode(openAP),
      forceSingleCore(false), isDualCore(false), detectedCores(1),
      networkTaskHandle(nullptr), commandQueue(nullptr), statusQueue(nullptr), statusMutex(nullptr),
      networkTaskRunning(false), lastNetworkTaskHealthCheck(0),
      cachedWiFiMode(WiFiMode::Offline), cachedConnected(false), cachedSignalStrength(-100), cachedIsAPMode(false) {
    initializeNVS();
    if (!this->openAPMode && strlen(this->apPassword) < 8) { 
        this->apPassword = DEFAULT_AP_PASSWORD; 
        this->openAPMode = false; 
        log("Warning: AP password too short for secure mode, using default: %s", DEFAULT_AP_PASSWORD); 
    }
    prefs.begin("devicehub", false);
    apSSID = generateAPSSID();
    mdnsHostname = generateMDNSHostname();
    currentNetworkStatus.wifiMode = WiFiMode::Offline;
    currentNetworkStatus.connected = false;
    currentNetworkStatus.signalStrength = -100;
    currentNetworkStatus.lastUpdate = 0;
    currentNetworkStatus.isAPMode = false;
    if (this->openAPMode) { 
        log("AP-only mode: Open (passwordless) Access Point enabled"); 
    } else {
        log("AP-only mode: Secured Access Point enabled"); 
    }
}

DeviceHub::~DeviceHub() {
    log("Cleaning up DeviceHub...");
    
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

void DeviceHub::begin(HardwareSerial& serial) {
    debugSerial = &serial;
    debugEnabled = true;
    log("DeviceHub::begin() called - Initializing core systems...");

#if defined(ESP32)
    // Initialize TCP/IP stack and default event loop ONCE.
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK) {
        log("FATAL: esp_netif_init() failed: %s", esp_err_to_name(err));
        return; // Cannot proceed
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) { // ESP_ERR_INVALID_STATE means it's already created
        log("FATAL: esp_event_loop_create_default() failed: %s", esp_err_to_name(err));
        return; // Cannot proceed
    }
#endif

    // UDP Sockets are bound here now, after netif/event loop is up.
    if (discoveryUdp.begin(DISCOVERY_PORT)) {
        log("Discovery UDP: Bound to port %d", DISCOVERY_PORT);
    } else {
        log("Discovery UDP: Failed to bind");
    }
    if (emergencyUdp.begin(EMERGENCY_PORT)) {
        log("Emergency UDP: Bound to port %d", EMERGENCY_PORT);
    } else {
        log("Emergency UDP: Failed to bind");
    }

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
    coap = new Coap(udp, 1500);
    log("Starting CoAP instance on port %d...", COAP_PORT);
    if (!coap->start(COAP_PORT)) {
        log("Error: Failed to start CoAP server instance!");
        return;
    } else {
        log("CoAP server instance started successfully.");
    }

    if (dtlsEnabled) {
        log("Creating/Starting Secure CoAP instance...");
        secureCoap = new Coap(udp, 1500);
        if (!secureCoap->start(SECURE_COAP_PORT)) { 
            log("Error: Failed to start Secure CoAP instance!");
        } else {
            log("Secure CoAP server instance started successfully.");
        }
    }
    
    setupCoAPResources();
    ensureDeviceUUID();
    
    // Only start OTA helper if we have WiFi credentials (not AP-only mode)
    if (!isAPOnlyMode() && strlen(ssid) > 0) {
        log("Starting OTA helper...");
        otaHelper.start(ssid, password, deviceName, password, 3232, 115200);
    } else {
        log("Skipping OTA helper (AP-only mode or no WiFi credentials)");
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

void DeviceHub::enableDebug(HardwareSerial& serial) {
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
    
    debugSerial->printf("[DeviceHub] %s\n", buffer);
}

void DeviceHub::loop() {
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
    log("Setting up CoAP server resources (Simplified Approach)...");

    // --- Handler for POST /v2/actions/resetPuzzle --- 
    coap->server([this](CoapPacket& packet, IPAddress ip, int port) {
        log("Callback fired for path: v2/actions");
        if (packet.code == COAP_POST) {
            log("Handling POST /v2/actions request...");
            JsonDocument doc;
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
    if (!doc["action"].isNull()) {
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
        JsonDocument doc;
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

void DeviceHub::handleIncomingPacket() {
    int packetSize = discoveryUdp.parsePacket();
    if (packetSize) {
        char incomingPacket[255];
        int len = discoveryUdp.read(incomingPacket, 255);
        if (len > 0) {
            incomingPacket[len] = 0;
        }
        Serial.printf("UDP packet received from %s:%d - %s\n", discoveryUdp.remoteIP().toString().c_str(), discoveryUdp.remotePort(), incomingPacket);

        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, incomingPacket);
        if (error) {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.f_str());
            return;
        }

        if (!doc["type"].isNull()) {
            String messageType = doc["type"].as<String>();

            if (messageType == "REQUEST_DEVICE_INFO") {
                log("Received REQUEST_DEVICE_INFO");
                sendDeviceInfo();
            }
        } else {
            Serial.println("Received message without type field");
        }
    }
}

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
            paramObj["unit"] = param.unit;
            paramObj["defaultValue"] = param.defaultValue;
            paramObj["min"] = param.min;
            paramObj["max"] = param.max;
        }
    }

    JsonArray eventsArray = capabilities["events"].to<JsonArray>();
    for (const auto& eventPair : events) {
        const Event& event = eventPair.second;
        JsonObject eventObj = eventsArray.add<JsonObject>();
        eventObj["id"] = event.id;
        eventObj["name"] = event.name;
        // Re-enable schema
        eventObj["dataSchema"] = event.dataSchema;
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

void DeviceHub::registerEvent(const String& eventId, const String& eventName, const JsonObject& schema) {
    Event event;
    event.id = eventId;
    event.name = eventName;
    event.dataSchema = schema;
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

WiFiClass DeviceHub::getWiFi() {
    return WiFi;
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
    log("Attempting WiFi connection to '%s'... (Timeout: %dms)", ssid, wifiTimeoutMs);
    
    // Ensure WiFi is in correct mode and disconnected first
    WiFi.mode(WIFI_OFF);
    delay(100);
    WiFi.mode(WIFI_STA);
    delay(100);
    
    // Configure WiFi with explicit settings to prevent connection errors
    WiFi.config(0U, 0U, 0U, 0U); // Use DHCP
    WiFi.setAutoConnect(false);
    WiFi.setAutoReconnect(false);
    
    // Begin connection with error checking
    wl_status_t status = WiFi.begin(ssid, password);
    if (status == WL_CONNECT_FAILED) {
        log("WiFi.begin() failed immediately with WL_CONNECT_FAILED");
        currentWiFiMode = WiFiMode::Offline;
        currentSignalStrength = -100;
        return false;
    }
    
    unsigned long startTime = millis();
    int attempts = 0;
    
    while (WiFi.status() != WL_CONNECTED && (millis() - startTime) < wifiTimeoutMs) {
        delay(500);
        if (debugEnabled && debugSerial) {
            debugSerial->print(".");
        }
        attempts++;
        
        // Check for specific error conditions
        wl_status_t currentStatus = WiFi.status();
        if (currentStatus == WL_CONNECT_FAILED || currentStatus == WL_CONNECTION_LOST) {
            log("\nWiFi connection failed with status: %d", currentStatus);
            break;
        }
        
        // Prevent infinite loop with reasonable attempt limit
        if (attempts > (wifiTimeoutMs / 500)) {
            log("\nWiFi connection timeout after %d attempts", attempts);
            break;
        }
    }
    
    wl_status_t finalStatus = WiFi.status();
    if (finalStatus == WL_CONNECTED) {
        currentWiFiMode = WiFiMode::Station;
        currentSignalStrength = WiFi.RSSI();
        log("\nWiFi connected successfully!");
        log("IP address: %s", WiFi.localIP().toString().c_str());
        log("Signal strength: %d dBm (%s)", currentSignalStrength, 
            getSignalQualityDescription(currentSignalStrength).c_str());
        log("MAC address: %s", WiFi.macAddress().c_str());
        log("Gateway: %s", WiFi.gatewayIP().toString().c_str());
        return true;
    } else {
        log("\nWiFi connection failed after %d attempts (%dms timeout).", attempts, wifiTimeoutMs);
        log("Final WiFi status: %d", finalStatus);
        
        // Provide specific error messages
        switch (finalStatus) {
            case WL_NO_SSID_AVAIL:
                log("Error: Network '%s' not found", ssid);
                break;
            case WL_CONNECT_FAILED:
                log("Error: Connection failed - check password");
                break;
            case WL_CONNECTION_LOST:
                log("Error: Connection was lost");
                break;
            case WL_DISCONNECTED:
                log("Error: WiFi disconnected");
                break;
            default:
                log("Error: Unknown WiFi error (status: %d)", finalStatus);
                break;
        }
        
        // Ensure WiFi is properly disconnected after failure
        WiFi.disconnect(true);
        currentWiFiMode = WiFiMode::Offline;
        currentSignalStrength = -100;
        return false;
    }
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
    stopCommand.data = "STOP";
    
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
    
    // Update initial status
    NetworkStatus status;
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
            if (command.data == "STOP") {
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
    command.data = data;
    command.payload = payload;
    
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
    return apModeForced && (strlen(ssid) == 0 || strcmp(ssid, "") == 0);
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

