# DeviceHub WiFi Connection Troubleshooting Guide

## Critical Issues Resolved

### 1. NVS Initialization Error ✅ FIXED!
**Problem**: `nvs_open failed: NOT_INITIALIZED`
**Solution**: ✅ **Now handled automatically by DeviceHub!**

~~Previously required manual initialization:~~
```cpp
// ❌ OLD WAY - No longer needed!
#include <nvs_flash.h>
esp_err_t err = nvs_flash_init();
// ... manual error handling
```

✅ **NEW WAY - Automatic!**
```cpp
#include <DeviceHub.h>

DeviceHub hub(ssid, password, "Device Name", "device_type");

void setup() {
    Serial.begin(115200);
    
    // NVS is automatically initialized by DeviceHub constructor!
    hub.begin(Serial);  // Just works!
}
```

**DeviceHub now automatically:**
- Initializes NVS in the constructor
- Handles all error conditions (corruption, version mismatch)
- Provides proper error logging
- No user code changes required!

### 2. FreeRTOS Queue Crash
**Problem**: `assert failed: xQueueGenericSend queue.c:822`
**Solution**: Fixed queue size for `xQueueOverwrite` usage

The status queue is now created with length 1 to properly support `xQueueOverwrite`.

### 3. WiFi Connection Error 0x3007
**Problem**: `connect failed! 0x3007` (Connection timeout)
**Solution**: Enhanced connection logic with proper error handling

## WiFi Connection Issues

### Common WiFi Error Codes:
- `0x3007`: Connection timeout
- `WL_NO_SSID_AVAIL` (1): Network not found
- `WL_CONNECT_FAILED` (4): Wrong password or authentication failed
- `WL_CONNECTION_LOST` (5): Connection lost during authentication

### Troubleshooting Steps:

#### 1. Verify Network Credentials
```cpp
// Double-check your WiFi credentials
const char* WIFI_SSID = "Newton";        // Exact network name
const char* WIFI_PASSWORD = "your_password";  // Exact password
```

#### 2. Check Network Compatibility
- Ensure 2.4GHz network (ESP32 doesn't support 5GHz)
- Check WPA2 security (WPA3 may cause issues)
- Verify network is broadcasting (not hidden)

#### 3. Test Basic WiFi Connection
```cpp
void testBasicWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin("Newton", "your_password");
    
    Serial.print("Connecting to WiFi");
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(1000);
        Serial.print(".");
        Serial.printf(" Status: %d", WiFi.status());
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.println("IP: " + WiFi.localIP().toString());
    } else {
        Serial.printf("\nConnection failed. Final status: %d\n", WiFi.status());
    }
}
```

#### 4. Network Scanning
```cpp
void scanNetworks() {
    Serial.println("Scanning for networks...");
    int n = WiFi.scanNetworks();
    
    for (int i = 0; i < n; ++i) {
        Serial.printf("%d: %s (%d) %s\n", 
            i + 1, 
            WiFi.SSID(i).c_str(), 
            WiFi.RSSI(i),
            WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "Open" : "Encrypted"
        );
    }
}
```

#### 5. Router/Network Issues
- **Distance**: Move closer to router
- **Interference**: Check for 2.4GHz interference (microwaves, Bluetooth)
- **Router overload**: Too many connected devices
- **MAC filtering**: Router may be blocking unknown devices
- **DHCP**: Router DHCP pool may be full

#### 6. ESP32 Hardware Issues
- **Power supply**: Ensure adequate 3.3V supply (500mA+)
- **Antenna**: Check for damaged or improperly connected antenna
- **Brown-out**: Add decoupling capacitors if power is unstable

## Quick Fixes

### 1. Minimal Working Example
```cpp
#include <WiFi.h>
#include <nvs_flash.h>

void setup() {
    Serial.begin(115200);
    
    // Initialize NVS
    nvs_flash_init();
    
    // Connect to WiFi
    WiFi.begin("Newton", "your_password");
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    
    Serial.println("Connected!");
    Serial.println(WiFi.localIP());
}

void loop() {
    delay(1000);
}
```

### 2. Alternative Network
Try connecting to a mobile hotspot to rule out router issues:
```cpp
const char* WIFI_SSID = "YourPhoneHotspot";
const char* WIFI_PASSWORD = "hotspot_password";
```

### 3. Single-Core Mode
If dual-core issues persist, force single-core mode:
```cpp
// Force single-core mode (last parameter = true)
DeviceHub deviceHub(ssid, password, device_name, device_type,
                    ap_password, timeout, enable_ap, check_interval, 
                    signal_strength, true);  // forceSingleCore = true
```

## DeviceHub Specific Solutions

### 1. Increase WiFi Timeout
```cpp
DeviceHub deviceHub(ssid, password, device_name, device_type,
                    "password123", 30000);  // 30 second timeout
```

### 2. Disable AP Fallback for Testing
```cpp
DeviceHub deviceHub(ssid, password, device_name, device_type,
                    "password123", 20000, false);  // Disable AP fallback
```

### 3. Force AP Mode for Setup
```cpp
void setup() {
    deviceHub.begin(Serial);
    deviceHub.forceAPMode();  // Skip WiFi connection, go straight to AP
}
```

## Network Diagnostics

### Check Your Router Settings:
1. **Network Name**: Ensure "Newton" is the exact SSID
2. **Password**: Verify case-sensitive password
3. **Security**: Use WPA2 (avoid WPA3 if possible)
4. **Channel**: Try changing to channel 1, 6, or 11
5. **Bandwidth**: Set to 20MHz (not 40MHz) for better compatibility

### ESP32 Recovery:
If the device keeps crashing:
1. **Full chip erase**: Use ESP32 flash tool to completely erase
2. **Fresh firmware**: Re-upload with latest Arduino ESP32 core
3. **Factory reset**: Hold BOOT button while resetting

## Expected Behavior After Fixes

With these fixes, you should see:
```
[DeviceHub] NVS initialized successfully
[DeviceHub] Dual-core mode activated
[DeviceHub] Network task started successfully
[DeviceHub] WiFi connected successfully!
[DeviceHub] IP address: 192.168.0.xxx
[DeviceHub] Signal strength: -xx dBm (Quality)
[DeviceHub] mDNS responder started: http://device-name.local
```

## Emergency Fallback

If all else fails, use this minimal initialization:
```cpp
#include <DeviceHub.h>
#include <nvs_flash.h>

DeviceHub deviceHub("Newton", "password", "Test Device", "controller");

void setup() {
    Serial.begin(115200);
    nvs_flash_init();
    
    // Force single-core, open AP mode for emergency access
    deviceHub = DeviceHub("Newton", "password", "Test Device", "controller",
                         "", 10000, true, 30000, -70, true, true);
    deviceHub.begin(Serial);
}
```

This will create an open access point you can connect to for debugging. 