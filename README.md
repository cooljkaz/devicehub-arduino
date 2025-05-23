# DeviceHub

DeviceHub is an Arduino library for managing IoT devices over Wi-Fi networks. It provides a robust interface for handling Wi-Fi connections, WebSocket communication, and device state management with support for actions, emergency protocols, and persistent data storage.

## Features

### Core Connectivity
- **Wi-Fi Management**: Easy setup and maintenance of Wi-Fi connections
- **WebSocket Communication**: Bidirectional real-time data exchange
- **UDP Broadcasting**: For device discovery on local networks
- **OTA Updates**: Over-the-air firmware updates

### Device Management
- **Action Registration**: Register custom actions that can be triggered remotely
- **Emergency Protocol**: Dedicated emergency communication channel
- **Device State Monitoring**: Track normal and emergency states
- **Message Acknowledgment**: Reliable message delivery with retry mechanism

### Data Handling
- **Persistent Storage**: Save and retrieve data across device reboots
- **JSON Processing**: Built-in JSON parsing and generation
- **Field Validation**: Validate incoming data fields against allowed lists

## Installation

1. Download the library as a ZIP file from the repository
2. Open the Arduino IDE
3. Go to `Sketch` > `Include Library` > `Add .ZIP Library`
4. Select the downloaded ZIP file
5. Restart the Arduino IDE

## Hardware Requirements

- ESP8266 or ESP32 microcontroller
- Stable power supply
- Connection to Wi-Fi network

## Usage

### Basic Setup

```cpp
#include <DeviceHub.h>

// Initialize with Wi-Fi credentials and device name
DeviceHub hub("YourWiFiSSID", "YourWiFiPassword", "MyDevice");

void setup() {
  Serial.begin(115200);
  
  // Initialize the DeviceHub
  hub.begin();
}

void loop() {
  // Must be called in the main loop to process incoming messages
  hub.loop();
}
```

### Registering Custom Actions

```cpp
// Define a callback function to handle the action
String handleLEDControl(const JsonObject& data) {
  int ledPin = data["pin"];
  bool state = data["state"];
  
  digitalWrite(ledPin, state ? HIGH : LOW);
  
  return "LED state changed";
}

void setup() {
  // ... other setup code ...
  
  // Register an action with specific allowed fields
  hub.registerAction("setLED", handleLEDControl, {"pin", "state"});
}
```

### Sending Messages

```cpp
// Send a status update
void sendStatus() {
  String message = "{\"temperature\": 25.5, \"humidity\": 68.2}";
  hub.sendMessage(message, "status_update");
}
```

### Using Persistent Storage

```cpp
// Save a configuration value
hub.savePersistentData("threshold", "25.5");

// Retrieve a configuration value
String threshold = hub.loadPersistentData("threshold", "20.0");  // Default is "20.0"
```

### Emergency Handling

```cpp
// Define an emergency handler
String handleEmergency(const JsonObject& data) {
  // Take emergency action
  return "Emergency handled";
}

void setup() {
  // ... other setup code ...
  
  // Register emergency action handler
  hub.registerEmergencyAction(handleEmergency);
}
```

## Examples

Check the `examples` folder for detailed usage examples:

- **BasicConnection**: Simple connection to Wi-Fi and message handling
- **CustomActions**: Registering and handling custom actions
- **EmergencyProtocol**: Setting up and using the emergency protocol
- **PersistentStorage**: Saving and loading persistent data
- **CompleteExample**: A comprehensive example showcasing all features

## Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

MIT License - see the LICENSE file for details.

## Troubleshooting

- Ensure your ESP8266/ESP32 has a stable power supply
- Check Wi-Fi signal strength in your deployment location
- For connection issues, verify your network credentials
- Serial monitor at 115200 baud rate can help debug connectivity problems
