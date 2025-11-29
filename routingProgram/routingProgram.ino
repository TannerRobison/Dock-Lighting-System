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

// MAC address mappings - UPDATE THESE WITH YOUR ACTUAL MACs
uint8_t control_station_1[] = {0x3C, 0x84, 0x27, 0xC4, 0x94, 0xF4}; 
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

// Callback when data is received (from control stations OR light stations)
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incoming_data, incomingData, sizeof(incoming_data));
  
  // Determine if this is from a control station or light station
  bool fromLightStation = (incoming_data.control_station_id > 10); // Light stations use IDs > 10
  
  if (fromLightStation) {
    // This is a response from a light station - forward to control station
    Serial.print("Light Station ");
    Serial.print(incoming_data.light_station_id);
    Serial.print(" responding to Control Station ");
    Serial.println(incoming_data.control_station_id);
    
    forwardToControlStation();
  } else {
    // This is a command from a control station - forward to light station
    Serial.print("Control Station ");
    Serial.print(incoming_data.control_station_id);
    Serial.print(" -> Light Station ");
    Serial.println(incoming_data.light_station_id);
    
    forwardToLightStation();
  }
}

void forwardToLightStation() {
  uint8_t* target_mac = getLightStationMAC(incoming_data.light_station_id);
  
  if (target_mac == nullptr) {
    Serial.println("Error: Invalid light station ID");
    return;
  }
  
  // Add peer and send
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, target_mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
  
  esp_err_t result = esp_now_send(target_mac, (uint8_t *) &incoming_data, sizeof(incoming_data));
  
  if (result == ESP_OK) {
    Serial.println("Forwarded to light station");
  } else {
    Serial.println("Error forwarding to light station");
  }
}

void forwardToControlStation() {
  uint8_t* target_mac = getControlStationMAC(incoming_data.control_station_id);
  
  if (target_mac == nullptr) {
    Serial.println("Error: Invalid control station ID");
    return;
  }
  
  // Add peer and send
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, target_mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
  
  esp_err_t result = esp_now_send(target_mac, (uint8_t *) &incoming_data, sizeof(incoming_data));
  
  if (result == ESP_OK) {
    Serial.println("Forwarded to control station");
  } else {
    Serial.println("Error forwarding to control station");
  }
}

void addAllPeers() {
  esp_now_peer_info_t peerInfo = {};
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  // Add control stations
  memcpy(peerInfo.peer_addr, control_station_1, 6);
  esp_now_add_peer(&peerInfo);
  
  memcpy(peerInfo.peer_addr, control_station_2, 6);
  esp_now_add_peer(&peerInfo);
  
  memcpy(peerInfo.peer_addr, control_station_3, 6);
  esp_now_add_peer(&peerInfo);
  
  // Add light stations
  memcpy(peerInfo.peer_addr, light_station_1, 6);
  esp_now_add_peer(&peerInfo);
  
  memcpy(peerInfo.peer_addr, light_station_2, 6);
  esp_now_add_peer(&peerInfo);
  
  memcpy(peerInfo.peer_addr, light_station_3, 6);
  esp_now_add_peer(&peerInfo);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  
  Serial.print("Router MAC Address: ");
  Serial.println(WiFi.macAddress());
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv);
  addAllPeers();
  
  Serial.println("ESP-NOW Bidirectional Router Ready!");
}

void loop() {
  delay(1000);
}
