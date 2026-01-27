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

// Dual-core configuration
#define DEVICEHUB_NETWORK_TASK_STACK_SIZE 8192
#define DEVICEHUB_NETWORK_TASK_PRIORITY 1
#define DEVICEHUB_NETWORK_CORE 0
#define DEVICEHUB_APPLICATION_CORE 1
#define DEVICEHUB_QUEUE_SIZE 10

// CoAP ports
#define COAP_PORT 5683
#define SECURE_COAP_PORT 5684
static const uint16_t EMERGENCY_PORT = 8890;

// CoAP content types (RFC 7252)
#define COAP_CONTENT_TYPE_LINK_FORMAT 40  // application/link-format (CoRE Link Format)
#define COAP_CONTENT_TYPE_JSON 50         // application/json

// AP Mode defaults
#define DEFAULT_AP_PASSWORD "devicehub123"
#define DEFAULT_WIFI_TIMEOUT_MS 20000
#define DEFAULT_CONNECTION_CHECK_INTERVAL_MS 30000
#define DEFAULT_SIGNAL_STRENGTH_CHECK_INTERVAL_MS 60000
#define DEFAULT_MIN_SIGNAL_STRENGTH -80  // dBm - below this will trigger reconnection attempt
#define DEFAULT_AP_OPEN_NETWORK false    // Default to password-protected AP
#define AP_IP_ADDRESS "192.168.4.1"
#define AP_GATEWAY "192.168.4.1"
#define AP_SUBNET "255.255.255.0"

// Include FreeRTOS headers for dual-core support
#if defined(ESP32)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#endif

enum class DeviceState { 
    Normal,
    Emergency,
    Idle,
    Active,
    Error
};

enum class WiFiMode {
    Station,      // Connected to WiFi network
    AccessPoint,  // Running as Access Point
    Offline       // No connection
};

// Environment types for multi-WiFi support
enum class WiFiEnvironment {
    Production,
    Development,
    Unknown
};

// WiFi network configuration for multi-network support
struct WiFiNetwork {
    String ssid;
    String password;
    WiFiEnvironment environment;
    int priority;  // Lower number = higher priority
};

struct ActionParameter {
    String name;
    String type;
    String unit;
    float defaultValue;
    float min;
    float max;
};

// Inter-core communication structures
// NOTE: Do NOT use String, JsonDocument, or other objects with internal heap pointers here!
// FreeRTOS queues use memcpy which breaks objects with dynamic memory.
// Use fixed-size char arrays instead.
struct NetworkCommand {
    enum Type {
        WIFI_RECONNECT,
        AP_MODE_SWITCH,
        SEND_DEVICE_INFO,
        EMIT_EVENT,
        FORCE_AP_MODE,
        CHECK_CONNECTION
    };
    Type type;
    char data[64];  // Fixed-size buffer instead of String
};

struct NetworkStatus {
    WiFiMode wifiMode;
    bool connected;
    int signalStrength;
    IPAddress currentIP;
    char lastError[64];  // Fixed-size buffer instead of String
    unsigned long lastUpdate;
    bool isAPMode;
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
        String dataSchemaJson;  // Serialized JSON string (stores a copy, not a reference)
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

    // Single constructor - uses networks configured via build defines (WIFI_1_SSID, etc.)
    DeviceHub(const char* deviceName);

    ~DeviceHub();

    // Configuration setters (call before begin())
    void setDeviceType(const char* type);           // Default: "esp32"
    void setEnableAPFallback(bool enable);          // Default: false
    void setAPPassword(const char* password);       // Default: "devicehub123"
    void setOpenAP(bool open);                      // Default: false (password-protected)
    void setWiFiTimeout(int timeoutMs);             // Default: 20000ms
    void setForceSingleCore(bool force);            // Default: false (auto-detect)
    void setProductionCheckInterval(unsigned long intervalMs);  // Default: 300000ms (5 min)
    void begin();
    void begin(Print& serial);
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
    
    // Message sending (v2 CoAP only, legacy methods retained for API compatibility)
    void sendMessage(const String& message, const String& type = "message");
    void sendEmergency(const String& message);
    void sendEmergencyResponse(const char* message);

    // Debug methods
    void enableDebug(Print& serial);
    void disableDebug();
    void log(const char* format, ...);

    WiFiClass getWiFi();

    // allow device to be discovered via mDNS using the <device name>.local domain
    void startMDNS();           
    static String toMdnsHost(String);     
    // Static members
    static const IPAddress BROADCAST_IP;
    static const IPAddress COAP_MULTICAST_IP; // 224.0.1.187

    // CoAP Helper
    String processRequest(const JsonObject& doc);
    void sendCoAPResponse(IPAddress ip, int port, uint16_t messageId, 
                          const String& payload, 
                          COAP_RESPONSE_CODE code,
                          COAP_CONTENT_TYPE type,
                          const uint8_t *token, 
                          int tokenlen);

    // New AP fallback methods
    bool isAPMode() const;                    // Returns true if in AP mode
    bool isConnected() const;                 // Returns true if WiFi connected
    String getAPSSID() const;                 // Get generated AP network name
    String getMDNSHostname() const;           // Get mDNS hostname (.local)
    IPAddress getIP() const;                  // Get device IP (WiFi or AP)
    void checkConnection();                   // Check/attempt reconnection
    void forceAPMode();                       // Force switch to AP mode
    WiFiMode getWiFiMode() const;            // Get current WiFi mode
    bool isAPOpen() const;                    // Returns true if AP is passwordless
    bool isAPOnlyMode() const;                // Returns true if device is in AP-only mode (no WiFi)

    // Enhanced connection monitoring methods
    int getSignalStrength() const;            // Get current WiFi signal strength (dBm)
    unsigned long getLastConnectionCheck() const;  // Get timestamp of last connection check
    bool isSignalWeak() const;                // Check if signal is below minimum threshold
    void setConnectionCheckInterval(int intervalMs);  // Configure check interval
    void setMinSignalStrength(int minStrength);       // Configure minimum signal strength

    // Dual-core status methods
    bool isDualCoreMode() const;              // Returns true if running in dual-core mode
    bool isSingleCoreMode() const;            // Returns true if running in single-core mode
    int getCoreCount() const;                 // Returns detected core count
    bool isNetworkTaskRunning() const;        // Returns true if network task is active

    // Multi-WiFi methods
    void addNetwork(const String& ssid, const String& password,
                    WiFiEnvironment env = WiFiEnvironment::Production, int priority = 99);
    void addNetwork(const String& ssid, const String& password,
                    const String& envStr, int priority = 99);
    void autoConfigureNetworks();             // Load networks from build defines
    WiFiEnvironment getCurrentEnvironment() const;
    String getCurrentEnvironmentString() const;
    bool isProduction() const;                // Returns true if connected to production network
    bool isDevelopment() const;               // Returns true if connected to development network
    String getConnectedSSID() const;          // Returns SSID of connected network
    const std::vector<WiFiNetwork>& getConfiguredNetworks() const;

private:
    const char* deviceName;
    const char* deviceType;
    String deviceUUID;
    DeviceState currentState;
    bool dtlsEnabled;
    Print* debugSerial;
    bool debugEnabled;

    WiFiUDP udp;
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

    // AP fallback configuration
    const char* apPassword;
    int wifiTimeoutMs;
    bool enableAPFallback;
    WiFiMode currentWiFiMode;
    String apSSID;
    String mdnsHostname;
    unsigned long lastConnectionCheck;
    unsigned long lastSignalCheck;
    int connectionCheckIntervalMs;
    int minSignalStrength;
    int currentSignalStrength;
    bool apModeForced;
    unsigned long totalReconnectionAttempts;
    unsigned long successfulReconnections;
    bool openAPMode;  // New: true for passwordless AP

    // Multi-WiFi support members
    std::vector<WiFiNetwork> configuredNetworks;
    WiFiEnvironment currentEnvironment;
    String connectedSSID;
    bool useMultiWiFi;  // True when using multi-network mode
    unsigned long productionCheckIntervalMs;  // How often to check for production network when on dev
    unsigned long lastProductionCheck;        // Last time we checked for production network

    // Dual-core support members
    bool forceSingleCore;
    bool isDualCore;
    int detectedCores;
    TaskHandle_t networkTaskHandle;
    QueueHandle_t commandQueue;
    QueueHandle_t statusQueue;
    SemaphoreHandle_t statusMutex;
    NetworkStatus currentNetworkStatus;
    bool networkTaskRunning;
    unsigned long lastNetworkTaskHealthCheck;
    
    // Thread-safe status cache (updated from network task)
    volatile WiFiMode cachedWiFiMode;
    volatile bool cachedConnected;
    volatile int cachedSignalStrength;
    volatile bool cachedIsAPMode;
    IPAddress cachedIP;

    void ensureDeviceUUID();
    void generateAndStoreUUID();
    bool isUUIDValid();
    void setupCoAPResources();
    void handleCoAPRequest(CoapPacket& packet, IPAddress ip, int port);
    String handleAction(const String& actionId, const JsonObject& payload, const String& correlationId = "");
    void notifyObservers(const String& eventId, const JsonObject& payload, const String& version);
    void sendCoAPMessage(const String& payload);
    void sendCoAPMessage(const String& payload, COAP_TYPE messageType);
    void resendPendingMessages();
    void sendPeriodicUpdate();
    void returnToNormalState();
    void sendAck(const String& messageId);
    void sendUdpMessage(const String& payload);

    void sendDeviceInfo();
    String getWellKnownCore();  // Returns CoRE Link Format for /.well-known/core

    void handleEmergencyPacket();
    void handleEmergencyStart();
    void handleEmergencyEnd();
  
    bool mdnsRunning = false;
    bool emergencyUdpInitialized = false;  // Track if UDP socket has been initialized

    // New private methods for AP functionality
    bool attemptWiFiConnection();
    void setupAccessPoint();
    void setupStationMode();
    void initializeUdpSockets();  // Initialize UDP sockets after TCP/IP stack is ready
    void setupMDNS();
    void checkAndReconnect();
    String generateAPSSID() const;
    String generateMDNSHostname() const;
    
    // Enhanced WiFi monitoring methods
    void monitorSignalStrength();
    String getSignalQualityDescription(int rssi) const;

    // Multi-WiFi helper methods
    WiFiEnvironment parseEnvironment(const String& envStr);
    bool tryConnectToNetwork(const WiFiNetwork& network, unsigned long timeoutMs);
    bool attemptMultiWiFiConnection();
    void scanAndSortNetworks();
    void checkForProductionNetwork();  // Switch to production if available when on dev

    // Dual-core specific methods
    bool detectAndSetupDualCore();
    void startNetworkTask();
    void stopNetworkTask();
    static void networkTaskFunction(void* parameter);
    void handleNetworkOperations();
    bool sendNetworkCommand(NetworkCommand::Type type, const String& data = "", const JsonDocument& payload = JsonDocument());
    void updateStatusFromNetworkTask(const NetworkStatus& status);
    void checkNetworkTaskHealth();
    bool initializeFreeRTOSObjects();
    void cleanupFreeRTOSObjects();
    
    // Thread-safe wrapper methods
    void safeUpdateWiFiMode(WiFiMode mode);
    void safeUpdateConnectionStatus(bool connected);
    void safeUpdateSignalStrength(int strength);
    void safeUpdateIP(IPAddress ip);
    
    // NVS initialization (handles ESP32 Non-Volatile Storage setup)
    void initializeNVS();

};

#endif // DEVICEHUB_H
