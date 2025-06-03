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
#define DISCOVERY_PORT 8888
static const uint16_t EMERGENCY_PORT = 8890;
static const uint16_t EMERGENCY_NOTIFICATION_PORT = 8891;

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

struct ActionParameter {
    String name;
    String type;
    String unit;
    float defaultValue;
    float min;
    float max;
};

// Inter-core communication structures
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
    String data;
    JsonDocument payload;
};

struct NetworkStatus {
    WiFiMode wifiMode;
    bool connected;
    int signalStrength;
    IPAddress currentIP;
    String lastError;
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

    // Enhanced constructor with dual-core support
    DeviceHub(const char* ssid, const char* password, const char* deviceName, const char* deviceType = "esp32");
    DeviceHub(const char* ssid, const char* password, const char* deviceName, const char* deviceType, 
              const char* apPassword, int wifiTimeoutMs = DEFAULT_WIFI_TIMEOUT_MS, bool enableAPFallback = true);
    DeviceHub(const char* ssid, const char* password, const char* deviceName, const char* deviceType, 
              const char* apPassword, int wifiTimeoutMs, bool enableAPFallback, 
              int connectionCheckIntervalMs, int minSignalStrength = DEFAULT_MIN_SIGNAL_STRENGTH);
    DeviceHub(const char* ssid, const char* password, const char* deviceName, const char* deviceType, 
              const char* apPassword, int wifiTimeoutMs, bool enableAPFallback, 
              int connectionCheckIntervalMs, int minSignalStrength, bool forceSingleCore);
    
    // New constructor with passwordless AP option
    DeviceHub(const char* ssid, const char* password, const char* deviceName, const char* deviceType, 
              const char* apPassword, int wifiTimeoutMs, bool enableAPFallback, 
              int connectionCheckIntervalMs, int minSignalStrength, bool forceSingleCore, bool openAP);
              
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

    WiFiClass getWiFi();

    // allow device to be discovered via mDNS using the <device name>.local domain
    void startMDNS();           
    static String toMdnsHost(String);     
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
    String handleAction(const String& actionId, const JsonObject& payload);
    void notifyObservers(const String& eventId, const JsonObject& payload, const String& version);
    void sendCoAPMessage(const String& payload);
    void sendCoAPMessage(const String& payload, COAP_TYPE messageType);
    void resendPendingMessages();
    void sendPeriodicUpdate();
    void returnToNormalState();
    void sendAck(const String& messageId);
    void sendUdpMessage(const String& payload);

    void handleIncomingPacket();
    void sendDeviceInfo();

    void handleEmergencyPacket();
    void handleEmergencyStart();
    void handleEmergencyEnd();
  
    bool mdnsRunning = false;

    // New private methods for AP functionality
    bool attemptWiFiConnection();
    void setupAccessPoint();
    void setupStationMode();
    void setupMDNS();
    void checkAndReconnect();
    String generateAPSSID() const;
    String generateMDNSHostname() const;
    
    // Enhanced WiFi monitoring methods
    void monitorSignalStrength();
    String getSignalQualityDescription(int rssi) const;

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
