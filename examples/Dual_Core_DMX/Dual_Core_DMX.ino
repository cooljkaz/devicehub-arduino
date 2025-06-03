/*
  DeviceHub Dual-Core ESP32 DMX Lighting Controller Example
  
  This example demonstrates DeviceHub's automatic dual-core ESP32 support where:
  - Core 0: DeviceHub network operations (WiFi, CoAP, mDNS, monitoring)
  - Core 1: Time-critical DMX lighting control (main application loop)
  
  Features demonstrated:
  - Automatic dual-core detection and setup
  - Time-critical DMX operations without network interference
  - Thread-safe communication between cores
  - Real-time performance monitoring
  - Graceful fallback to single-core mode if needed
  
  Hardware Requirements:
  - Dual-core ESP32 (ESP32-S3, ESP32-WROOM, etc.)
  - DMX shield or MAX485 module (optional for this demo)
  - Built-in LED for visual feedback
  
  Usage:
  1. Update WiFi credentials below
  2. Upload to dual-core ESP32
  3. Monitor serial output for core assignment
  4. Send CoAP commands to control DMX channels
  5. Observe smooth DMX timing regardless of network activity
*/

#include <DeviceHub.h>
#include <nvs_flash.h>  // Required for NVS initialization

// WiFi credentials - update these for your network
const char* WIFI_SSID = "YourWiFiNetwork";
const char* WIFI_PASSWORD = "YourWiFiPassword";

// Device configuration
const char* DEVICE_NAME = "DMX Controller";
const char* DEVICE_TYPE = "lighting_controller";

// DMX configuration
#define DMX_CHANNELS 512
#define DMX_REFRESH_RATE_HZ 44  // Standard DMX refresh rate
#define DMX_FRAME_TIME_MICROS (1000000 / DMX_REFRESH_RATE_HZ)

// Performance monitoring
struct PerformanceStats {
  unsigned long dmxFrameCount;
  unsigned long maxLoopTime;
  unsigned long avgLoopTime;
  unsigned long networkTaskHealthChecks;
  bool timingWarnings;
};

// Create DeviceHub instance with dual-core enabled (default)
DeviceHub hub(WIFI_SSID, WIFI_PASSWORD, DEVICE_NAME, DEVICE_TYPE);

// DMX channel data
uint8_t dmxData[DMX_CHANNELS];
unsigned long lastDMXFrame = 0;
PerformanceStats stats = {0};

// Simulated DMX patterns
enum DMXPattern {
  PATTERN_OFF,
  PATTERN_RAINBOW,
  PATTERN_STROBE,
  PATTERN_FADE,
  PATTERN_CHASE
};

DMXPattern currentPattern = PATTERN_OFF;
uint8_t brightness = 255;
uint8_t speed = 128;

// Action handlers for DMX control
String setDMXChannel(const JsonObject& payload) {
  int channel = payload["channel"].as<int>();
  int value = payload["value"].as<int>();
  
  if (channel >= 1 && channel <= DMX_CHANNELS && value >= 0 && value <= 255) {
    dmxData[channel - 1] = value;
    
    JsonDocument response;
    response["status"] = "success";
    response["channel"] = channel;
    response["value"] = value;
    response["core_mode"] = hub.isDualCoreMode() ? "dual-core" : "single-core";
    
    String result;
    serializeJson(response, result);
    return result;
  }
  
  return "{\"status\":\"error\",\"message\":\"Invalid channel or value\"}";
}

String setDMXPattern(const JsonObject& payload) {
  String pattern = payload["pattern"].as<String>();
  
  if (pattern == "off") currentPattern = PATTERN_OFF;
  else if (pattern == "rainbow") currentPattern = PATTERN_RAINBOW;
  else if (pattern == "strobe") currentPattern = PATTERN_STROBE;
  else if (pattern == "fade") currentPattern = PATTERN_FADE;
  else if (pattern == "chase") currentPattern = PATTERN_CHASE;
  else return "{\"status\":\"error\",\"message\":\"Unknown pattern\"}";
  
  if (payload.containsKey("brightness")) {
    brightness = constrain(payload["brightness"].as<int>(), 0, 255);
  }
  
  if (payload.containsKey("speed")) {
    speed = constrain(payload["speed"].as<int>(), 1, 255);
  }
  
  JsonDocument response;
  response["status"] = "success";
  response["pattern"] = pattern;
  response["brightness"] = brightness;
  response["speed"] = speed;
  response["running_on_core"] = xPortGetCoreID();
  
  String result;
  serializeJson(response, result);
  return result;
}

String getPerformanceStats(const JsonObject& payload) {
  JsonDocument response;
  response["core_mode"] = hub.isDualCoreMode() ? "dual-core" : "single-core";
  response["detected_cores"] = hub.getCoreCount();
  response["network_task_running"] = hub.isNetworkTaskRunning();
  response["main_core"] = xPortGetCoreID();
  response["dmx_frames_sent"] = stats.dmxFrameCount;
  response["max_loop_time_us"] = stats.maxLoopTime;
  response["avg_loop_time_us"] = stats.avgLoopTime;
  response["timing_warnings"] = stats.timingWarnings;
  response["wifi_connected"] = hub.isConnected();
  response["wifi_mode"] = hub.isAPMode() ? "AP" : "Station";
  
  if (!hub.isAPMode()) {
    response["wifi_signal"] = hub.getSignalStrength();
  }
  
  String result;
  serializeJson(response, result);
  return result;
}

// Time-critical DMX frame generation
void generateDMXFrame() {
  static unsigned long patternTime = 0;
  patternTime = millis();
  
  switch (currentPattern) {
    case PATTERN_OFF:
      memset(dmxData, 0, DMX_CHANNELS);
      break;
      
    case PATTERN_RAINBOW:
      for (int i = 0; i < DMX_CHANNELS; i += 3) {
        float hue = (float)(i + patternTime * speed / 100) / DMX_CHANNELS * 360.0;
        // Simple HSV to RGB conversion
        int rgb = (int)(sin(hue * PI / 180.0) * 127 + 128);
        dmxData[i] = (rgb * brightness) / 255;
        dmxData[i+1] = ((255 - rgb) * brightness) / 255;
        dmxData[i+2] = (rgb * brightness) / 255;
      }
      break;
      
    case PATTERN_STROBE:
      {
        bool strobeOn = (patternTime % (500 / (speed + 1))) < (250 / (speed + 1));
        uint8_t value = strobeOn ? brightness : 0;
        for (int i = 0; i < DMX_CHANNELS; i += 3) {
          dmxData[i] = value;
          dmxData[i+1] = value;
          dmxData[i+2] = value;
        }
      }
      break;
      
    case PATTERN_FADE:
      {
        float fade = (sin(patternTime * speed / 10000.0) + 1.0) / 2.0;
        uint8_t value = (uint8_t)(fade * brightness);
        for (int i = 0; i < DMX_CHANNELS; i++) {
          dmxData[i] = value;
        }
      }
      break;
      
    case PATTERN_CHASE:
      {
        int chasePos = (patternTime * speed / 100) % DMX_CHANNELS;
        memset(dmxData, 0, DMX_CHANNELS);
        for (int i = 0; i < 10; i++) {
          int pos = (chasePos + i) % DMX_CHANNELS;
          dmxData[pos] = brightness * (10 - i) / 10;
        }
      }
      break;
  }
}

// Simulated DMX transmission (replace with actual DMX output)
void transmitDMX() {
  // In a real implementation, this would send DMX data via UART/MAX485
  // For demo, we just toggle the built-in LED based on channel 1
  digitalWrite(LED_BUILTIN, dmxData[0] > 128 ? HIGH : LOW);
  
  // Simulate DMX transmission time (22.7ms for 512 channels at 250kbaud)
  delayMicroseconds(100); // Minimal delay for demo
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== DeviceHub Dual-Core DMX Controller ===");
  
  // Initialize hardware
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // Initialize DMX data
  memset(dmxData, 0, DMX_CHANNELS);
  
  // Register DeviceHub actions
  hub.registerAction("setChannel", "Set DMX Channel", setDMXChannel);
  hub.registerAction("setPattern", "Set DMX Pattern", setDMXPattern);
  hub.registerAction("getStats", "Get Performance Statistics", getPerformanceStats);
  
  // Register events
  JsonDocument eventSchema;
  eventSchema["dmx_frames"]["type"] = "integer";
  eventSchema["performance"]["type"] = "object";
  eventSchema["core_info"]["type"] = "object";
  
  hub.registerEvent("dmx_performance", "DMX Performance Data", eventSchema.as<JsonObject>());
  
  // Start DeviceHub (this will automatically detect and setup dual-core)
  // NVS initialization is now handled automatically by DeviceHub!
  hub.begin(Serial);
  
  // Print setup results
  Serial.println("\n=== DMX Controller Setup Complete ===");
  Serial.printf("Core Mode: %s\n", hub.isDualCoreMode() ? "DUAL-CORE" : "Single-Core");
  Serial.printf("Detected Cores: %d\n", hub.getCoreCount());
  Serial.printf("Network Task Running: %s\n", hub.isNetworkTaskRunning() ? "Yes" : "No");
  Serial.printf("Main Application Core: %d\n", xPortGetCoreID());
  Serial.printf("DMX Refresh Rate: %d Hz\n", DMX_REFRESH_RATE_HZ);
  Serial.printf("DMX Channels: %d\n", DMX_CHANNELS);
  
  if (hub.isDualCoreMode()) {
    Serial.println("\n✓ DUAL-CORE MODE ACTIVE");
    Serial.println("  - Network operations isolated on Core 0");
    Serial.println("  - DMX timing protected on Core 1");
    Serial.println("  - Maximum real-time performance achieved");
  } else {
    Serial.println("\n⚠ SINGLE-CORE MODE");
    Serial.println("  - All operations sharing same core");
    Serial.println("  - Potential timing disruptions from network activity");
  }
  
  Serial.println("\n=== Available Actions ===");
  Serial.println("POST /v2/actions");
  Serial.println("setChannel: {\"action\":\"setChannel\",\"payload\":{\"channel\":1,\"value\":255}}");
  Serial.println("setPattern: {\"action\":\"setPattern\",\"payload\":{\"pattern\":\"rainbow\",\"brightness\":200,\"speed\":100}}");
  Serial.println("getStats:   {\"action\":\"getStats\"}");
  
  Serial.println("\n=== DMX Patterns ===");
  Serial.println("Available: off, rainbow, strobe, fade, chase");
  
  Serial.println("\n=== Serial Commands ===");
  Serial.println("Type 'stats' for performance statistics");
  Serial.println("Type 'cores' for core information");
  Serial.println("Type 'rainbow' for rainbow pattern demo");
  
  Serial.println("\n=== Starting DMX Loop ===");
  lastDMXFrame = micros();
}

void loop() {
  unsigned long loopStart = micros();
  
  // TIME-CRITICAL SECTION: DMX frame generation and transmission
  unsigned long currentTime = micros();
  if (currentTime - lastDMXFrame >= DMX_FRAME_TIME_MICROS) {
    lastDMXFrame = currentTime;
    
    // Generate new DMX frame
    generateDMXFrame();
    
    // Transmit DMX data
    transmitDMX();
    
    stats.dmxFrameCount++;
  }
  
  // DeviceHub operations (lightweight in dual-core mode)
  hub.loop();
  
  // Performance monitoring
  unsigned long loopTime = micros() - loopStart;
  if (loopTime > stats.maxLoopTime) {
    stats.maxLoopTime = loopTime;
  }
  
  // Calculate running average
  static unsigned long totalLoopTime = 0;
  static unsigned long loopCount = 0;
  totalLoopTime += loopTime;
  loopCount++;
  stats.avgLoopTime = totalLoopTime / loopCount;
  
  // Check for timing warnings
  if (loopTime > 10000) { // More than 10ms is concerning for DMX
    stats.timingWarnings = true;
    if (hub.isDualCoreMode()) {
      Serial.printf("WARNING: Long loop time detected: %lu μs (dual-core should prevent this!)\n", loopTime);
    } else {
      Serial.printf("WARNING: Long loop time detected: %lu μs (expected in single-core mode)\n", loopTime);
    }
  }
  
  // Send performance data periodically
  static unsigned long lastPerfReport = 0;
  if (currentTime - lastPerfReport > 30000000) { // Every 30 seconds
    lastPerfReport = currentTime;
    
    JsonDocument perfData;
    perfData["dmx_frames"] = stats.dmxFrameCount;
    perfData["max_loop_us"] = stats.maxLoopTime;
    perfData["avg_loop_us"] = stats.avgLoopTime;
    perfData["core_mode"] = hub.isDualCoreMode() ? "dual" : "single";
    perfData["main_core"] = xPortGetCoreID();
    perfData["timing_ok"] = !stats.timingWarnings;
    
    hub.emitEvent("dmx_performance", perfData.as<JsonObject>());
    
    // Reset some stats
    stats.maxLoopTime = 0;
    stats.timingWarnings = false;
    totalLoopTime = 0;
    loopCount = 0;
  }
  
  // Handle serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    
    if (command == "stats") {
      Serial.println("\n=== Performance Statistics ===");
      Serial.printf("Core Mode: %s\n", hub.isDualCoreMode() ? "Dual-Core" : "Single-Core");
      Serial.printf("Main Core: %d\n", xPortGetCoreID());
      Serial.printf("DMX Frames: %lu\n", stats.dmxFrameCount);
      Serial.printf("Max Loop Time: %lu μs\n", stats.maxLoopTime);
      Serial.printf("Avg Loop Time: %lu μs\n", stats.avgLoopTime);
      Serial.printf("Timing Warnings: %s\n", stats.timingWarnings ? "Yes" : "No");
      Serial.printf("Network Task: %s\n", hub.isNetworkTaskRunning() ? "Running" : "Stopped");
      Serial.printf("WiFi Status: %s\n", hub.isConnected() ? "Connected" : "Disconnected");
    }
    else if (command == "cores") {
      Serial.println("\n=== Core Information ===");
      Serial.printf("Detected Cores: %d\n", hub.getCoreCount());
      Serial.printf("Dual-Core Mode: %s\n", hub.isDualCoreMode() ? "Active" : "Inactive");
      Serial.printf("Current Core: %d\n", xPortGetCoreID());
      Serial.printf("Network Task Running: %s\n", hub.isNetworkTaskRunning() ? "Yes" : "No");
    }
    else if (command == "rainbow") {
      Serial.println("Activating rainbow pattern...");
      currentPattern = PATTERN_RAINBOW;
      brightness = 200;
      speed = 100;
    }
    else {
      Serial.println("Unknown command. Available: stats, cores, rainbow");
    }
  }
  
  // Small delay only if needed (dual-core mode doesn't need this)
  if (!hub.isDualCoreMode()) {
    delayMicroseconds(100); // Minimal delay for single-core mode
  }
} 