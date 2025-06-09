/*
  DeviceHub AP-Only Mode Example
  
  This example demonstrates DeviceHub's AP-only mode where the device starts
  directly as a WiFi Access Point without attempting to connect to any WiFi network.
  This is useful for devices that should always be standalone access points.
  
  Features demonstrated:
  - Direct AP mode startup (no WiFi connection attempts)
  - Open (passwordless) or secured AP options
  - Automatic mDNS setup for easy access
  - CoAP server for device communication
  - Web interface accessible via AP
  
  Hardware Requirements:
  - ESP32 or ESP8266
  
  Usage:
  1. Upload the sketch
  2. Device immediately creates AP named "my-iot-device"
  3. Connect to the AP (password: "mydevice123" for secure mode)
  4. Access device at http://192.168.4.1 or http://my-iot-device.local
  5. Use CoAP commands or web interface
*/

#include <DeviceHub.h>

// Device configuration
const char* DEVICE_NAME = "My IoT Device";
const char* DEVICE_TYPE = "sensor";
const char* AP_PASSWORD = "mydevice123";  // Must be 8+ characters for secure AP

// Choose AP mode type:
// true = Open (passwordless) AP - easier for users but less secure
// false = Secured AP with password - more secure but requires password entry
const bool OPEN_AP_MODE = false;

// Create DeviceHub instance in AP-only mode
// This constructor forces AP mode and never attempts WiFi connection
DeviceHub hub(DEVICE_NAME, DEVICE_TYPE, AP_PASSWORD, OPEN_AP_MODE);

// Device state for demonstration
struct DeviceData {
  float temperature;
  float humidity;
  bool ledState;
  int brightness;
} deviceData;

// Action callbacks
String toggleLED(const JsonObject& payload) {
  deviceData.ledState = !deviceData.ledState;
  digitalWrite(LED_BUILTIN, deviceData.ledState ? HIGH : LOW);
  
  JsonDocument response;
  response["led_state"] = deviceData.ledState ? "on" : "off";
  response["message"] = "LED toggled successfully";
  response["mode"] = "ap_only";
  
  String result;
  serializeJson(response, result);
  return result;
}

String setBrightness(const JsonObject& payload) {
  if (payload.containsKey("brightness")) {
    int newBrightness = payload["brightness"].as<int>();
    if (newBrightness >= 0 && newBrightness <= 255) {
      deviceData.brightness = newBrightness;
      
      // In a real application, you might control PWM here
      analogWrite(LED_BUILTIN, deviceData.brightness);
      
      JsonDocument response;
      response["brightness"] = deviceData.brightness;
      response["message"] = "Brightness set successfully";
      response["mode"] = "ap_only";
      
      String result;
      serializeJson(response, result);
      return result;
    }
  }
  
  return "{\"error\":\"Invalid brightness value. Must be 0-255.\"}";
}

String getDeviceStatus(const JsonObject& payload) {
  JsonDocument response;
  response["temperature"] = deviceData.temperature;
  response["humidity"] = deviceData.humidity;
  response["led_state"] = deviceData.ledState ? "on" : "off";
  response["brightness"] = deviceData.brightness;
  response["mode"] = "ap_only";
  response["ap_ssid"] = hub.getAPSSID();
  response["ap_ip"] = hub.getIP().toString();
  response["ap_open"] = hub.isAPOpen();
  response["uptime"] = millis();
  
  String result;
  serializeJson(response, result);
  return result;
}

void sendSensorData() {
  static unsigned long lastSensorRead = 0;
  
  if (millis() - lastSensorRead > 10000) { // Every 10 seconds
    lastSensorRead = millis();
    
    // Simulate sensor readings
    deviceData.temperature = 20.0 + (random(-50, 50) / 10.0);
    deviceData.humidity = 45.0 + (random(-100, 100) / 10.0);
    
    JsonDocument sensorData;
    sensorData["temperature"] = deviceData.temperature;
    sensorData["humidity"] = deviceData.humidity;
    sensorData["led_state"] = deviceData.ledState;
    sensorData["brightness"] = deviceData.brightness;
    sensorData["mode"] = "ap_only";
    sensorData["timestamp"] = millis();
    
    hub.emitEvent("sensor_reading", sensorData.as<JsonObject>());
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== DeviceHub AP-Only Mode Example ===");
  
  // Initialize hardware
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // Initialize device data
  deviceData.temperature = 22.5;
  deviceData.humidity = 45.0;
  deviceData.ledState = false;
  deviceData.brightness = 128;
  
  // Register actions
  hub.registerAction("toggleLED", "Toggle LED", toggleLED);
  hub.registerAction("setBrightness", "Set LED Brightness", setBrightness);
  hub.registerAction("getStatus", "Get Device Status", getDeviceStatus);
  
  // Register events
  JsonDocument sensorSchema;
  sensorSchema["temperature"]["type"] = "number";
  sensorSchema["temperature"]["unit"] = "celsius";
  sensorSchema["humidity"]["type"] = "number";
  sensorSchema["humidity"]["unit"] = "percent";
  sensorSchema["led_state"]["type"] = "string";
  sensorSchema["brightness"]["type"] = "integer";
  sensorSchema["mode"]["type"] = "string";
  sensorSchema["timestamp"]["type"] = "integer";
  
  hub.registerEvent("sensor_reading", "Sensor Data", sensorSchema.as<JsonObject>());
  
  // Start DeviceHub in AP-only mode
  hub.begin(Serial);
  
  // Print setup results
  Serial.println("\n=== AP-Only Mode Setup Complete ===");
  Serial.printf("Device Mode: %s\n", hub.isAPOnlyMode() ? "AP-ONLY" : "Standard");
  Serial.printf("AP Mode: %s\n", hub.isAPMode() ? "Active" : "Inactive");
  Serial.printf("AP Type: %s\n", hub.isAPOpen() ? "Open (No Password)" : "Secured");
  Serial.printf("AP SSID: %s\n", hub.getAPSSID().c_str());
  
  if (!hub.isAPOpen()) {
    Serial.printf("AP Password: %s\n", AP_PASSWORD);
  }
  
  Serial.printf("AP IP Address: %s\n", hub.getIP().toString().c_str());
  Serial.printf("mDNS Hostname: %s.local\n", hub.getMDNSHostname().c_str());
  
  Serial.println("\n=== How to Connect ===");
  if (hub.isAPOpen()) {
    Serial.printf("1. Connect to WiFi network '%s' (no password required)\n", hub.getAPSSID().c_str());
  } else {
    Serial.printf("1. Connect to WiFi network '%s'\n", hub.getAPSSID().c_str());
    Serial.printf("   Password: %s\n", AP_PASSWORD);
  }
  Serial.printf("2. Open browser to http://192.168.4.1\n");
  Serial.printf("3. Or use mDNS: http://%s.local\n", hub.getMDNSHostname().c_str());
  
  Serial.println("\n=== Available Actions (CoAP) ===");
  Serial.println("POST /v2/actions");
  Serial.println("Examples:");
  Serial.println("  {\"action\":\"toggleLED\"}");
  Serial.println("  {\"action\":\"setBrightness\",\"payload\":{\"brightness\":128}}");
  Serial.println("  {\"action\":\"getStatus\"}");
  
  Serial.println("\n=== Available Events ===");
  Serial.println("Event: sensor_reading (every 10 seconds)");
  
  Serial.println("\nDevice ready - Access Point active!");
}

void loop() {
  // Run DeviceHub
  hub.loop();
  
  // Send periodic sensor data
  sendSensorData();
  
  // Simple LED breathing effect based on brightness setting
  static unsigned long lastBreathingUpdate = 0;
  static bool breathingUp = true;
  static int currentPWM = 0;
  
  if (millis() - lastBreathingUpdate > 50) { // Update every 50ms
    lastBreathingUpdate = millis();
    
    if (deviceData.ledState) {
      // Breathing effect
      if (breathingUp) {
        currentPWM += 5;
        if (currentPWM >= deviceData.brightness) {
          breathingUp = false;
          currentPWM = deviceData.brightness;
        }
      } else {
        currentPWM -= 5;
        if (currentPWM <= 20) {
          breathingUp = true;
          currentPWM = 20;
        }
      }
      analogWrite(LED_BUILTIN, currentPWM);
    } else {
      analogWrite(LED_BUILTIN, 0);
    }
  }
} 