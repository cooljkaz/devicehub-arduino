/*
  WiFi Access Point Fallback Example with Enhanced Monitoring
  
  This example demonstrates DeviceHub's automatic WiFi Access Point fallback feature
  with enhanced WiFi connection monitoring including signal strength tracking.
  
  Features demonstrated:
  - Automatic WiFi connection with AP fallback
  - Enhanced WiFi monitoring with signal strength tracking
  - Configurable connection check intervals
  - Automatic reconnection on weak signals
  - Connection statistics and monitoring
  - mDNS .local domain access in both modes
  - Manual AP mode switching
  
  Hardware Requirements:
  - ESP32 or ESP8266
  
  Usage:
  1. Update WiFi credentials below
  2. Upload the sketch
  3. Monitor serial output for connection status and signal monitoring
  4. If WiFi fails, device creates AP named "my-iot-device" 
  5. Connect to AP and access http://my-iot-device.local
  6. If WiFi succeeds, access http://my-iot-device.local on your network
*/

#include <DeviceHub.h>

// WiFi credentials - update these for your network
const char* WIFI_SSID = "YourWiFiNetwork";
const char* WIFI_PASSWORD = "YourWiFiPassword";

// Device configuration
const char* DEVICE_NAME = "My IoT Device";
const char* DEVICE_TYPE = "sensor";

// Enhanced AP fallback configuration
const char* AP_PASSWORD = "mydevice123";     // Minimum 8 characters for WPA2
const int WIFI_TIMEOUT = 15000;              // 15 seconds timeout
const bool ENABLE_AP_FALLBACK = true;
const int CONNECTION_CHECK_INTERVAL = 45000; // Check every 45 seconds
const int MIN_SIGNAL_STRENGTH = -75;         // Reconnect if signal below -75 dBm

// Create DeviceHub instance with enhanced WiFi monitoring
DeviceHub hub(WIFI_SSID, WIFI_PASSWORD, DEVICE_NAME, DEVICE_TYPE, 
              AP_PASSWORD, WIFI_TIMEOUT, ENABLE_AP_FALLBACK,
              CONNECTION_CHECK_INTERVAL, MIN_SIGNAL_STRENGTH);

// For demonstration - a simple action
String toggleLED(const JsonObject& payload) {
  static bool ledState = false;
  ledState = !ledState;
  digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
  
  JsonDocument response;
  response["led_state"] = ledState ? "on" : "off";
  response["message"] = "LED toggled successfully";
  response["wifi_signal"] = hub.getSignalStrength();
  response["wifi_mode"] = hub.isAPMode() ? "ap" : "station";
  
  String result;
  serializeJson(response, result);
  return result;
}

// Enhanced sensor data with WiFi monitoring info
void sendSensorData() {
  static unsigned long lastSensorRead = 0;
  static int sensorCount = 0;
  
  if (millis() - lastSensorRead > 30000) { // Every 30 seconds
    lastSensorRead = millis();
    
    JsonDocument sensorData;
    sensorData["temperature"] = 23.5 + (random(-50, 50) / 10.0); // Simulated temperature
    sensorData["humidity"] = 45.0 + (random(-100, 100) / 10.0);  // Simulated humidity
    sensorData["count"] = ++sensorCount;
    sensorData["wifi_mode"] = hub.isAPMode() ? "ap" : "station";
    sensorData["wifi_connected"] = hub.isConnected();
    
    if (!hub.isAPMode()) {
      sensorData["wifi_signal"] = hub.getSignalStrength();
      sensorData["wifi_ip"] = hub.getIP().toString();
      sensorData["signal_weak"] = hub.isSignalWeak();
    }
    
    hub.emitEvent("sensor_reading", sensorData.as<JsonObject>());
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== DeviceHub Enhanced WiFi Monitoring Example ===");
  
  // Initialize built-in LED for demonstration
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // Register actions
  hub.registerAction("toggleLED", "Toggle LED", toggleLED);
  
  // Register events
  JsonDocument sensorSchema;
  sensorSchema["temperature"]["type"] = "number";
  sensorSchema["temperature"]["unit"] = "celsius";
  sensorSchema["humidity"]["type"] = "number";
  sensorSchema["humidity"]["unit"] = "percent";
  sensorSchema["count"]["type"] = "integer";
  sensorSchema["wifi_mode"]["type"] = "string";
  sensorSchema["wifi_connected"]["type"] = "boolean";
  sensorSchema["wifi_signal"]["type"] = "integer";
  sensorSchema["wifi_ip"]["type"] = "string";
  sensorSchema["signal_weak"]["type"] = "boolean";
  
  hub.registerEvent("sensor_reading", "Enhanced Sensor Data", sensorSchema.as<JsonObject>());
  
  // Start DeviceHub with debug output
  hub.begin(Serial);
  
  // Print final status
  Serial.println("\n=== Connection Status ===");
  Serial.printf("WiFi Mode: %s\n", hub.isAPMode() ? "Access Point" : "Station");
  Serial.printf("Connected: %s\n", hub.isConnected() ? "Yes" : "No");
  Serial.printf("IP Address: %s\n", hub.getIP().toString().c_str());
  Serial.printf("mDNS Hostname: %s.local\n", hub.getMDNSHostname().c_str());
  
  if (hub.isAPMode()) {
    Serial.printf("AP SSID: %s\n", hub.getAPSSID().c_str());
    Serial.printf("AP Password: %s\n", AP_PASSWORD);
    Serial.println("\nTo connect:");
    Serial.printf("1. Connect to WiFi network '%s'\n", hub.getAPSSID().c_str());
    Serial.printf("2. Open browser to http://%s.local\n", hub.getMDNSHostname().c_str());
  } else {
    Serial.printf("Signal Strength: %d dBm\n", hub.getSignalStrength());
    Serial.printf("Signal Weak: %s\n", hub.isSignalWeak() ? "Yes" : "No");
    Serial.printf("\nDevice available at: http://%s.local\n", hub.getMDNSHostname().c_str());
  }
  
  Serial.println("\n=== WiFi Monitoring Configuration ===");
  Serial.printf("Connection Check Interval: %d ms\n", CONNECTION_CHECK_INTERVAL);
  Serial.printf("Minimum Signal Strength: %d dBm\n", MIN_SIGNAL_STRENGTH);
  Serial.printf("WiFi Timeout: %d ms\n", WIFI_TIMEOUT);
  
  Serial.println("\n=== Available Actions ===");
  Serial.println("POST /v2/actions");
  Serial.println("Body: {\"action\":\"toggleLED\"}");
  
  Serial.println("\n=== Available Events ===");
  Serial.println("Event: sensor_reading (every 30 seconds with WiFi info)");
  
  Serial.println("\n=== Serial Commands ===");
  Serial.println("Type 'status' to check connection status");
  Serial.println("Type 'signal' to check signal strength");
  Serial.println("Type 'ap' to force AP mode");
  Serial.println("Type 'reconnect' to check/force reconnection");
  Serial.println("Type 'config' to show monitoring configuration");
}

void loop() {
  // Run DeviceHub
  hub.loop();
  
  // Send sensor data periodically
  sendSensorData();
  
  // Handle serial commands for demonstration
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    
    if (command == "status") {
      Serial.println("\n=== Current Status ===");
      Serial.printf("WiFi Mode: %s\n", hub.isAPMode() ? "Access Point" : "Station");
      Serial.printf("Connected: %s\n", hub.isConnected() ? "Yes" : "No");
      Serial.printf("IP Address: %s\n", hub.getIP().toString().c_str());
      Serial.printf("mDNS: http://%s.local\n", hub.getMDNSHostname().c_str());
      Serial.printf("Last Connection Check: %lu ms ago\n", millis() - hub.getLastConnectionCheck());
      
      if (hub.isAPMode()) {
        Serial.printf("AP SSID: %s\n", hub.getAPSSID().c_str());
        Serial.printf("Connected clients: %d\n", WiFi.softAPgetStationNum());
      } else {
        Serial.printf("WiFi Signal: %d dBm\n", hub.getSignalStrength());
        Serial.printf("Signal Weak: %s\n", hub.isSignalWeak() ? "Yes" : "No");
        Serial.printf("WiFi Status: %d\n", WiFi.status());
      }
    }
    else if (command == "signal") {
      if (hub.isAPMode()) {
        Serial.println("Signal monitoring not available in AP mode");
      } else {
        int signal = hub.getSignalStrength();
        Serial.printf("WiFi Signal: %d dBm (%s)\n", signal, 
                     hub.isSignalWeak() ? "WEAK" : "OK");
        Serial.printf("Threshold: %d dBm\n", MIN_SIGNAL_STRENGTH);
      }
    }
    else if (command == "ap") {
      Serial.println("Forcing AP mode...");
      hub.forceAPMode();
    }
    else if (command == "reconnect") {
      Serial.println("Checking connection...");
      hub.checkConnection();
    }
    else if (command == "config") {
      Serial.println("\n=== Monitoring Configuration ===");
      Serial.printf("Connection Check Interval: %d ms\n", CONNECTION_CHECK_INTERVAL);
      Serial.printf("Signal Strength Threshold: %d dBm\n", MIN_SIGNAL_STRENGTH);
      Serial.printf("WiFi Connection Timeout: %d ms\n", WIFI_TIMEOUT);
      Serial.printf("AP Fallback Enabled: %s\n", ENABLE_AP_FALLBACK ? "Yes" : "No");
    }
    else {
      Serial.println("Unknown command. Available: status, signal, ap, reconnect, config");
    }
  }
} 