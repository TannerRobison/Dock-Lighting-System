#include <esp_now.h>
#include <WiFi.h>

// Data structure for communication
typedef struct struct_message {
  int control_station_id;
  int light_station_id;
  uint8_t pwmValue;
} struct_message;

// Create data structure instance
struct_message incoming_data;
struct_message outgoing_data;

// MAC address mappings (replace with your actual MAC addresses)
uint8_t control_station_1[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t control_station_2[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t control_station_3[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t light_station_1[] = {0x3C, 0x84, 0x27, 0xC3, 0xD3, 0x64};
uint8_t light_station_2[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t light_station_3[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Function to get MAC address from control station ID
uint8_t* getControlStationMAC(int control_id) {
  switch(control_id) {
    case 1: return control_station_1;
    case 2: return control_station_2;
    case 3: return control_station_3;
    default: return nullptr;
  }
}

// Function to get MAC address from light station ID
uint8_t* getLightStationMAC(int light_id) {
  switch(light_id) {
    case 1: return light_station_1;
    case 2: return light_station_2;
    case 3: return light_station_3;
    default: return nullptr;
  }
}

// Callback when data is received from control stations
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incoming_data, incomingData, sizeof(incoming_data));
  
  Serial.print("Received from Control Station: ");
  Serial.println(incoming_data.control_station_id);
  Serial.print("Target Light Station: ");
  Serial.println(incoming_data.light_station_id);
  Serial.print("PWM Value: ");
  Serial.println(incoming_data.pwmValue);
  
  // Forward the message to the appropriate light station
  forwardToLightStation();
}

// Callback when data is sent to light stations
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status to Light Station: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// Forward received data to the target light station
void forwardToLightStation() {
  uint8_t* target_mac = getLightStationMAC(incoming_data.light_station_id);
  
  if (target_mac == nullptr) {
    Serial.println("Error: Invalid light station ID");
    return;
  }
  
  // Prepare outgoing data
  outgoing_data.control_station_id = incoming_data.control_station_id;
  outgoing_data.light_station_id = incoming_data.light_station_id;
  outgoing_data.pwmValue = incoming_data.pwmValue;
  
  // Add peer if not already added
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, target_mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  esp_now_add_peer(&peerInfo);
  
  // Send data
  esp_err_t result = esp_now_send(target_mac, (uint8_t *) &outgoing_data, sizeof(outgoing_data));
  
  if (result == ESP_OK) {
    Serial.println("Forwarding data to light station...");
  } else {
    Serial.println("Error sending data to light station");
  }
}

void setup() {
  Serial.begin(115200);
  
  // Set device as WiFi Station
  WiFi.mode(WIFI_STA);
  
  // Print MAC address (useful for setting up other devices)
  Serial.print("Router MAC Address: ");
  Serial.println(WiFi.macAddress());
  
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register callbacks
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  
  // Add all control stations as peers (for receiving)
  addControlStationsAsPeers();
  
  Serial.println("ESP-NOW Router Hub Ready!");
  Serial.println("Waiting for control station commands...");
}

// Add all known control stations as peers
void addControlStationsAsPeers() {
  // Add control station 1
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, control_station_1, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
  
  // Add control station 2
  memcpy(peerInfo.peer_addr, control_station_2, 6);
  esp_now_add_peer(&peerInfo);
  
  // Add control station 3
  memcpy(peerInfo.peer_addr, control_station_3, 6);
  esp_now_add_peer(&peerInfo);
}

void loop() {
  // Main loop doesn't need to do much - everything is callback driven
  delay(1000);
}
