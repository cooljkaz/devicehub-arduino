/*
  DMX Controller with WiFi AP Fallback Example (Without DeviceHub)
  
  This example demonstrates how to add WiFi Access Point fallback to an existing
  DMX controller application that doesn't use DeviceHub.
  
  Features:
  - WiFi connection with automatic AP fallback
  - Basic web interface for DMX control
  - mDNS support for easy access
  - Connection monitoring and retry logic
  
  Hardware Requirements:
  - ESP32
  - DMX shield or MAX485 module (optional for this demo)
  
  Usage:
  1. Update WiFi credentials below
  2. Upload to ESP32
  3. If WiFi fails, device creates AP named "DMX-Controller"
  4. Connect to AP and access http://192.168.4.1 or http://dmx-controller.local
*/

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <ESPAsyncWebServer.h>

// WiFi credentials - update these for your network
const char* WIFI_SSID = "YourWiFiNetwork";
const char* WIFI_PASSWORD = "YourWiFiPassword";

// Device configuration
const char* DEVICE_NAME = "DMX Controller";
const char* AP_SSID = "DMX-Controller";
const char* AP_PASSWORD = "dmxcontrol123";  // Minimum 8 characters

// WiFi configuration
const int WIFI_TIMEOUT_MS = 15000;
const bool ENABLE_AP_FALLBACK = true;
const int CONNECTION_CHECK_INTERVAL_MS = 30000;

// DMX configuration
#define DMX_CHANNELS 512
uint8_t dmxData[DMX_CHANNELS];

// Network state
enum WiFiMode {
  WIFI_MODE_STATION,
  WIFI_MODE_AP,
  WIFI_MODE_OFFLINE
};

WiFiMode currentWiFiMode = WIFI_MODE_OFFLINE;
unsigned long lastConnectionCheck = 0;
AsyncWebServer server(80);

// Forward declarations
bool attemptWiFiConnection();
void setupAccessPoint();
void setupWebServer();
void setupMDNS();
void checkAndReconnect();
void handleWebInterface();

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== DMX Controller with WiFi AP Fallback ===");
  
  // Initialize DMX data
  memset(dmxData, 0, DMX_CHANNELS);
  
  // Set some demo values
  dmxData[0] = 100;  // Channel 1
  dmxData[1] = 150;  // Channel 2
  dmxData[2] = 200;  // Channel 3
  
  Serial.println("Initializing WiFi...");
  
  // Try WiFi connection first
  if (attemptWiFiConnection()) {
    Serial.println("WiFi connected successfully!");
    currentWiFiMode = WIFI_MODE_STATION;
    Serial.printf("IP address: %s\n", WiFi.localIP().toString().c_str());
  } else if (ENABLE_AP_FALLBACK) {
    Serial.println("WiFi connection failed, starting Access Point...");
    setupAccessPoint();
    currentWiFiMode = WIFI_MODE_AP;
  } else {
    Serial.println("WiFi connection failed and AP fallback disabled.");
    currentWiFiMode = WIFI_MODE_OFFLINE;
  }
  
  // Setup mDNS and web server if we have network connectivity
  if (currentWiFiMode != WIFI_MODE_OFFLINE) {
    setupMDNS();
    setupWebServer();
    
    Serial.println("\n=== Network Setup Complete ===");
    if (currentWiFiMode == WIFI_MODE_STATION) {
      Serial.printf("Mode: WiFi Station\n");
      Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
      Serial.printf("Access via: http://%s.local\n", "dmx-controller");
    } else {
      Serial.printf("Mode: WiFi Access Point\n");
      Serial.printf("SSID: %s\n", AP_SSID);
      Serial.printf("Password: %s\n", AP_PASSWORD);
      Serial.printf("IP: %s\n", WiFi.softAPIP().toString().c_str());
      Serial.printf("Connect to '%s' network and access http://192.168.4.1\n", AP_SSID);
    }
  }
  
  Serial.println("Setup complete - DMX Controller ready");
  Serial.println("Web interface available for DMX control");
}

void loop() {
  // Check WiFi connection periodically
  if (millis() - lastConnectionCheck > CONNECTION_CHECK_INTERVAL_MS) {
    lastConnectionCheck = millis();
    checkAndReconnect();
  }
  
  // Handle web server (this is automatically handled by ESPAsyncWebServer)
  
  // DMX processing would go here
  // For demo, just cycle some values
  static unsigned long lastDMXUpdate = 0;
  if (millis() - lastDMXUpdate > 100) {  // Update every 100ms
    lastDMXUpdate = millis();
    
    // Simple demo: slowly cycle channel 1
    static uint8_t brightness = 0;
    static bool increasing = true;
    
    if (increasing) {
      brightness += 2;
      if (brightness >= 250) increasing = false;
    } else {
      brightness -= 2;
      if (brightness <= 5) increasing = true;
    }
    
    dmxData[0] = brightness;
    
    // In a real application, you would transmit DMX data here
    // transmitDMX();
  }
  
  // Small delay to prevent watchdog issues
  delay(1);
}

bool attemptWiFiConnection() {
  Serial.printf("Attempting WiFi connection to '%s'...\n", WIFI_SSID);
  
  // Ensure clean start
  WiFi.mode(WIFI_OFF);
  delay(100);
  WiFi.mode(WIFI_STA);
  delay(100);
  
  // Configure WiFi
  WiFi.setAutoConnect(false);
  WiFi.setAutoReconnect(false);
  
  // Begin connection
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startTime) < WIFI_TIMEOUT_MS) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
    return true;
  } else {
    Serial.printf("WiFi connection failed (Status: %d)\n", WiFi.status());
    WiFi.disconnect();
    return false;
  }
}

void setupAccessPoint() {
  Serial.println("Setting up WiFi Access Point...");
  
  // Stop any existing connections
  WiFi.disconnect();
  WiFi.mode(WIFI_AP);
  
  // Configure AP with custom IP
  IPAddress local_ip(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  
  if (!WiFi.softAPConfig(local_ip, gateway, subnet)) {
    Serial.println("AP config failed");
    return;
  }
  
  // Start the access point
  if (WiFi.softAP(AP_SSID, AP_PASSWORD)) {
    Serial.println("Access Point started successfully!");
    Serial.printf("SSID: %s\n", AP_SSID);
    Serial.printf("Password: %s\n", AP_PASSWORD);
    Serial.printf("IP address: %s\n", WiFi.softAPIP().toString().c_str());
  } else {
    Serial.println("Failed to start Access Point!");
  }
}

void setupMDNS() {
  String hostname = "dmx-controller";
  
  if (MDNS.begin(hostname.c_str())) {
    Serial.printf("mDNS responder started: http://%s.local\n", hostname.c_str());
    
    // Add services
    MDNS.addService("http", "tcp", 80);
    MDNS.addServiceTxt("http", "tcp", "device", "dmx_controller");
    MDNS.addServiceTxt("http", "tcp", "type", "lighting");
  } else {
    Serial.println("Failed to start mDNS responder");
  }
}

void setupWebServer() {
  // Serve root page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<!DOCTYPE html><html><head><title>DMX Controller</title>";
    html += "<style>body{font-family:Arial;margin:40px;} .channel{margin:10px 0;} input[type=range]{width:300px;}</style>";
    html += "</head><body>";
    html += "<h1>DMX Controller</h1>";
    html += "<h2>Network Status</h2>";
    html += "<p>Mode: " + String(currentWiFiMode == WIFI_MODE_STATION ? "WiFi Station" : "Access Point") + "</p>";
    html += "<p>IP: " + (currentWiFiMode == WIFI_MODE_STATION ? WiFi.localIP().toString() : WiFi.softAPIP().toString()) + "</p>";
    html += "<h2>DMX Channels</h2>";
    
    // Show first 8 channels with sliders
    for (int i = 0; i < 8; i++) {
      html += "<div class='channel'>";
      html += "Channel " + String(i + 1) + ": ";
      html += "<input type='range' min='0' max='255' value='" + String(dmxData[i]) + "' ";
      html += "onchange='setChannel(" + String(i + 1) + ", this.value)'> ";
      html += "<span id='ch" + String(i + 1) + "'>" + String(dmxData[i]) + "</span>";
      html += "</div>";
    }
    
    html += "<script>";
    html += "function setChannel(ch, val) {";
    html += "  fetch('/set?ch=' + ch + '&val=' + val);";
    html += "  document.getElementById('ch' + ch).innerText = val;";
    html += "}";
    html += "</script>";
    html += "</body></html>";
    
    request->send(200, "text/html", html);
  });
  
  // Handle channel setting
  server.on("/set", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("ch") && request->hasParam("val")) {
      int channel = request->getParam("ch")->value().toInt();
      int value = request->getParam("val")->value().toInt();
      
      if (channel >= 1 && channel <= DMX_CHANNELS && value >= 0 && value <= 255) {
        dmxData[channel - 1] = value;
        Serial.printf("Set Channel %d to %d\n", channel, value);
        request->send(200, "text/plain", "OK");
      } else {
        request->send(400, "text/plain", "Invalid parameters");
      }
    } else {
      request->send(400, "text/plain", "Missing parameters");
    }
  });
  
  // Status endpoint
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = "{";
    json += "\"mode\":\"" + String(currentWiFiMode == WIFI_MODE_STATION ? "station" : "ap") + "\",";
    json += "\"ip\":\"" + (currentWiFiMode == WIFI_MODE_STATION ? WiFi.localIP().toString() : WiFi.softAPIP().toString()) + "\",";
    json += "\"channels\":[";
    for (int i = 0; i < 8; i++) {
      if (i > 0) json += ",";
      json += String(dmxData[i]);
    }
    json += "]}";
    
    request->send(200, "application/json", json);
  });
  
  server.begin();
  Serial.println("Web server started");
}

void checkAndReconnect() {
  if (currentWiFiMode == WIFI_MODE_STATION) {
    // Check if WiFi is still connected
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi connection lost! Attempting reconnection...");
      
      if (attemptWiFiConnection()) {
        Serial.println("WiFi reconnected successfully!");
      } else if (ENABLE_AP_FALLBACK) {
        Serial.println("WiFi reconnection failed, switching to AP mode...");
        setupAccessPoint();
        currentWiFiMode = WIFI_MODE_AP;
        // Note: You may need to restart the web server for AP mode
        // depending on your implementation
      }
    } else {
      // Connection is good
      int signal = WiFi.RSSI();
      Serial.printf("WiFi OK - IP: %s, Signal: %d dBm\n", 
                   WiFi.localIP().toString().c_str(), signal);
    }
  } else if (currentWiFiMode == WIFI_MODE_AP) {
    // In AP mode, periodically try to reconnect to WiFi
    static unsigned long lastReconnectAttempt = 0;
    if (millis() - lastReconnectAttempt > 300000) {  // Try every 5 minutes
      lastReconnectAttempt = millis();
      Serial.println("Periodic attempt to reconnect to WiFi from AP mode...");
      
      if (attemptWiFiConnection()) {
        Serial.println("Successfully reconnected to WiFi!");
        WiFi.softAPdisconnect();  // Stop AP mode
        currentWiFiMode = WIFI_MODE_STATION;
        setupMDNS();  // Restart mDNS for station mode
      }
    }
    
    // Log AP status
    int connectedClients = WiFi.softAPgetStationNum();
    Serial.printf("AP mode - Connected clients: %d\n", connectedClients);
  }
} 