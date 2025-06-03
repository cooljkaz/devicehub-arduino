/*
  DeviceHub Open Access Point Example
  
  This example demonstrates DeviceHub's passwordless AP mode for easy device setup.
  When WiFi connection fails, the device creates an OPEN (no password) access point
  that anyone can connect to for initial configuration.
  
  Features demonstrated:
  - Passwordless AP mode for easy setup
  - Automatic WiFi fallback to open network
  - Clear connection instructions
  - Secure vs open mode comparison
  
  Security Note:
  - Open networks should only be used for initial setup
  - Consider switching to secure mode after configuration
  - Monitor connected clients in production
  
  Hardware Requirements:
  - ESP32 (any variant)
  - Serial monitor for setup instructions
  
  Usage:
  1. Upload to ESP32
  2. Device will try to connect to WiFi
  3. If WiFi fails, creates open "device-setup" network
  4. Connect any device to this network (no password)
  5. Access http://device-setup.local for configuration
*/

#include <DeviceHub.h>

// WiFi credentials - will fallback to open AP if these fail
const char* WIFI_SSID = "YourWiFiNetwork";       // Update with your WiFi
const char* WIFI_PASSWORD = "YourWiFiPassword";   // Update with your password

// Device configuration
const char* DEVICE_NAME = "Device Setup";
const char* DEVICE_TYPE = "configuration_device";

// Create DeviceHub with passwordless AP mode
// Parameters: ssid, password, deviceName, deviceType, apPassword, timeout, fallback, checkInterval, signalMin, singleCore, openAP
DeviceHub hub(WIFI_SSID, WIFI_PASSWORD, DEVICE_NAME, DEVICE_TYPE,
              "",      // AP password (ignored for open networks)
              15000,   // 15 second WiFi timeout
              true,    // Enable AP fallback
              30000,   // Check connection every 30 seconds
              -80,     // Minimum signal strength
              false,   // Use dual-core if available
              true);   // Enable OPEN AP mode (no password)

// Action handlers for device configuration
String getNetworkInfo(const JsonObject& payload) {
  JsonDocument response;
  response["wifi_ssid"] = WIFI_SSID;
  response["wifi_connected"] = hub.isConnected();
  response["wifi_mode"] = hub.isAPMode() ? "Access Point" : "Station";
  response["ap_open"] = hub.isAPOpen();
  response["ip_address"] = hub.getIP().toString();
  response["device_name"] = DEVICE_NAME;
  response["mdns_hostname"] = hub.getMDNSHostname();
  
  if (!hub.isAPMode()) {
    response["wifi_signal"] = hub.getSignalStrength();
    response["signal_quality"] = hub.getSignalStrength() > -60 ? "Good" : "Weak";
  } else {
    response["connected_clients"] = WiFi.softAPgetStationNum();
  }
  
  String result;
  serializeJson(response, result);
  return result;
}

String configureWiFi(const JsonObject& payload) {
  // In a real application, you would:
  // 1. Save new WiFi credentials to preferences
  // 2. Restart the device to try new credentials
  // 3. Fallback to open AP if new credentials fail
  
  String newSSID = payload["ssid"].as<String>();
  String newPassword = payload["password"].as<String>();
  
  JsonDocument response;
  
  if (newSSID.length() > 0) {
    response["status"] = "success";
    response["message"] = "WiFi credentials received. Device will restart to apply new settings.";
    response["new_ssid"] = newSSID;
    response["note"] = "In production, these would be saved and device restarted";
  } else {
    response["status"] = "error";
    response["message"] = "SSID is required";
  }
  
  String result;
  serializeJson(response, result);
  return result;
}

String getSecurityStatus(const JsonObject& payload) {
  JsonDocument response;
  response["ap_mode"] = hub.isAPMode();
  response["ap_open"] = hub.isAPOpen();
  response["security_level"] = hub.isAPOpen() ? "OPEN - No password required" : "WPA2 - Password protected";
  response["recommendation"] = hub.isAPOpen() ? "Switch to secure mode after setup" : "Secure mode active";
  
  if (hub.isAPMode()) {
    response["connected_clients"] = WiFi.softAPgetStationNum();
    response["ap_ssid"] = hub.getAPSSID();
  }
  
  String result;
  serializeJson(response, result);
  return result;
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== DeviceHub Open Access Point Example ===");
  
  // Register configuration actions
  hub.registerAction("getNetworkInfo", "Get Network Information", getNetworkInfo);
  hub.registerAction("configureWiFi", "Configure WiFi Credentials", configureWiFi);
  hub.registerAction("getSecurityStatus", "Get Security Status", getSecurityStatus);
  
  // Register events
  JsonDocument eventSchema;
  eventSchema["network_status"]["type"] = "object";
  eventSchema["security_info"]["type"] = "object";
  
  hub.registerEvent("network_change", "Network Status Changed", eventSchema.as<JsonObject>());
  hub.registerEvent("security_alert", "Security Status Alert", eventSchema.as<JsonObject>());
  
  // Start DeviceHub
  hub.begin(Serial);
  
  // Print setup results
  Serial.println("\n=== Device Setup Complete ===");
  Serial.printf("WiFi Connection Attempted: %s\n", WIFI_SSID);
  Serial.printf("Current Mode: %s\n", hub.isAPMode() ? "Access Point" : "WiFi Station");
  
  if (hub.isAPMode()) {
    Serial.printf("AP Security: %s\n", hub.isAPOpen() ? "OPEN (No Password)" : "WPA2 Protected");
    Serial.printf("AP Network: %s\n", hub.getAPSSID().c_str());
    
    if (hub.isAPOpen()) {
      Serial.println("\n🔓 OPEN ACCESS POINT ACTIVE");
      Serial.println("┌─────────────────────────────────────────┐");
      Serial.println("│            SETUP INSTRUCTIONS          │");
      Serial.println("├─────────────────────────────────────────┤");
      Serial.printf("│ 1. Connect to '%s' network    │\n", hub.getAPSSID().c_str());
      Serial.println("│    (No password required)              │");
      Serial.println("│                                         │");
      Serial.printf("│ 2. Open: http://%s.local          │\n", hub.getMDNSHostname().c_str());
      Serial.printf("│    Or:    http://%s             │\n", hub.getIP().toString().c_str());
      Serial.println("│                                         │");
      Serial.println("│ 3. Use these CoAP actions:              │");
      Serial.println("│    - getNetworkInfo                     │");
      Serial.println("│    - configureWiFi                     │");
      Serial.println("│    - getSecurityStatus                 │");
      Serial.println("└─────────────────────────────────────────┘");
      Serial.println("\n⚠️  SECURITY WARNING:");
      Serial.println("   Open network - anyone nearby can connect!");
      Serial.println("   Switch to secure mode after initial setup.");
    }
  } else {
    Serial.printf("WiFi Connected: %s\n", hub.getIP().toString().c_str());
    Serial.printf("Signal Strength: %d dBm\n", hub.getSignalStrength());
    Serial.printf("Access via: http://%s.local\n", hub.getMDNSHostname().c_str());
  }
  
  Serial.println("\n=== Available CoAP Actions ===");
  Serial.println("POST to: coap://device-setup.local/v2/actions");
  Serial.println("getNetworkInfo: {\"action\":\"getNetworkInfo\"}");
  Serial.println("configureWiFi:  {\"action\":\"configureWiFi\",\"payload\":{\"ssid\":\"MyWiFi\",\"password\":\"mypass\"}}");
  Serial.println("getSecurityStatus: {\"action\":\"getSecurityStatus\"}");
  
  Serial.println("\n=== Monitoring Started ===");
}

void loop() {
  static unsigned long lastStatusCheck = 0;
  static bool lastAPMode = false;
  static bool lastOpenMode = false;
  
  // DeviceHub operations
  hub.loop();
  
  // Monitor for network changes
  unsigned long currentTime = millis();
  if (currentTime - lastStatusCheck > 10000) { // Check every 10 seconds
    lastStatusCheck = currentTime;
    
    bool currentAPMode = hub.isAPMode();
    bool currentOpenMode = hub.isAPOpen();
    
    // Check for mode changes
    if (currentAPMode != lastAPMode || currentOpenMode != lastOpenMode) {
      Serial.println("\n📡 Network Status Changed!");
      Serial.printf("Mode: %s\n", currentAPMode ? "Access Point" : "WiFi Station");
      
      if (currentAPMode) {
        Serial.printf("Security: %s\n", currentOpenMode ? "OPEN" : "WPA2");
        Serial.printf("Network: %s\n", hub.getAPSSID().c_str());
        Serial.printf("Connected Clients: %d\n", WiFi.softAPgetStationNum());
      } else {
        Serial.printf("WiFi: %s\n", WIFI_SSID);
        Serial.printf("IP: %s\n", hub.getIP().toString().c_str());
        Serial.printf("Signal: %d dBm\n", hub.getSignalStrength());
      }
      
      // Emit network change event
      JsonDocument eventData;
      eventData["ap_mode"] = currentAPMode;
      eventData["open_network"] = currentOpenMode;
      eventData["ip_address"] = hub.getIP().toString();
      eventData["timestamp"] = millis();
      
      hub.emitEvent("network_change", eventData.as<JsonObject>());
      
      lastAPMode = currentAPMode;
      lastOpenMode = currentOpenMode;
    }
    
    // Security monitoring for open networks
    if (currentAPMode && currentOpenMode) {
      int connectedClients = WiFi.softAPgetStationNum();
      if (connectedClients > 0) {
        static int lastClientCount = 0;
        if (connectedClients != lastClientCount) {
          Serial.printf("⚠️  Open network: %d clients connected\n", connectedClients);
          
          // Emit security alert
          JsonDocument alertData;
          alertData["alert_type"] = "open_network_access";
          alertData["connected_clients"] = connectedClients;
          alertData["timestamp"] = millis();
          alertData["message"] = "Clients connected to open network";
          
          hub.emitEvent("security_alert", alertData.as<JsonObject>());
          
          lastClientCount = connectedClients;
        }
      }
    }
  }
  
  // Handle serial commands for testing
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    
    if (command == "status") {
      Serial.println("\n=== Current Status ===");
      Serial.printf("Mode: %s\n", hub.isAPMode() ? "Access Point" : "WiFi Station");
      Serial.printf("Connected: %s\n", hub.isConnected() ? "Yes" : "No");
      Serial.printf("IP: %s\n", hub.getIP().toString().c_str());
      
      if (hub.isAPMode()) {
        Serial.printf("Security: %s\n", hub.isAPOpen() ? "OPEN" : "WPA2");
        Serial.printf("Clients: %d\n", WiFi.softAPgetStationNum());
      } else {
        Serial.printf("Signal: %d dBm\n", hub.getSignalStrength());
      }
    }
    else if (command == "help") {
      Serial.println("\n=== Serial Commands ===");
      Serial.println("status - Show current network status");
      Serial.println("help   - Show this help message");
    }
    else {
      Serial.println("Unknown command. Type 'help' for available commands.");
    }
  }
} 