#include <DeviceHub.h>

// WiFi credentials
const char* ssid = "your_ssid";
const char* password = "your_password";

// Device information
const char* deviceName = "coap_example";
const char* deviceType = "sensor";

// Create DeviceHub instance
DeviceHub hub(ssid, password, deviceName, deviceType);

// Example action parameters
std::vector<ActionParameter> ledParams = {
    {"state", "boolean", "", 0, 0, 1},
    {"brightness", "number", "%", 100, 0, 100}
};

void setup() {
    Serial.begin(115200);
    
    // Register actions with parameters
    hub.registerAction("toggle_led", "Toggle LED", [](const JsonObject& payload) {
        bool state = payload["state"] | false;
        int brightness = payload["brightness"] | 100;
        
        // Implement LED control here
        Serial.printf("Setting LED: state=%d, brightness=%d%%\n", state, brightness);
        
        // Return success response
        StaticJsonDocument<256> response;
        response["status"] = "success";
        response["state"] = state;
        response["brightness"] = brightness;
        
        String output;
        serializeJson(response, output);
        return output;
    }, ledParams);
    
    // Register a simple action without parameters
    hub.registerAction("get_status", [](const JsonObject& payload) {
        StaticJsonDocument<256> response;
        response["status"] = "online";
        response["uptime"] = millis() / 1000;
        response["free_memory"] = ESP.getFreeHeap();
        
        String output;
        serializeJson(response, output);
        return output;
    });
    
    // Register an event with data schema
    StaticJsonDocument<256> eventSchema;
    eventSchema["temperature"] = "number";
    eventSchema["humidity"] = "number";
    eventSchema["pressure"] = "number";
    hub.registerEvent("environment", "Environment Data", eventSchema.as<JsonObject>());
    
    // Register an event with no schema
    hub.registerEvent("puzzle_solved", "Puzzle Solved");
    
    // Register emergency action
    hub.registerEmergencyAction([](const JsonObject& payload) {
        Serial.println("Emergency action triggered!");
        return "{\"status\":\"emergency_handled\"}";
    });
    
    // Register reset action
    hub.registerResetAction([](const JsonObject& payload) {
        Serial.println("Reset action triggered!");
        return "{\"status\":\"reset_complete\"}";
    });
    
    // Initialize the hub
    hub.begin();
    
    Serial.println("DeviceHub CoAP example started");
    Serial.printf("Device UUID: %s\n", hub.getDeviceUUID().c_str());
}

void loop() {
    hub.loop();
    
    // Example: Emit environment event every 5 seconds
    static unsigned long lastEvent = 0;
    if (millis() - lastEvent > 5000) {
        StaticJsonDocument<256> payload;
        payload["temperature"] = random(20, 30);
        payload["humidity"] = random(40, 60);
        payload["pressure"] = random(980, 1020);
        
        // Emit event using v2 format
        hub.emitEvent("environment", payload.as<JsonObject>());
        lastEvent = millis();
    }
    
    // Example: Emit status event every 10 seconds
    static unsigned long lastStatusEvent = 0;
    if (millis() - lastStatusEvent > 10000) {
        StaticJsonDocument<256> statusPayload;
        statusPayload["status"] = "running";
        statusPayload["uptime"] = millis() / 1000;
        statusPayload["free_memory"] = ESP.getFreeHeap();
        
        // Emit event with no predefined schema
        hub.emitEvent("device_status", statusPayload.as<JsonObject>());
        lastStatusEvent = millis();
    }
    
    // Example: Send emergency message if button pressed
    if (digitalRead(0) == LOW) {  // Assuming button on GPIO 0
        hub.sendEmergency("Button pressed - manual emergency trigger");
        delay(1000);  // Debounce
    }
} 