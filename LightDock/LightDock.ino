#include <esp_now.h>
#include <WiFi.h>

const int pwmPin = D4;
const int MY_LIGHT_STATION_ID = 1; // Change this to 2, 3, etc.

typedef struct struct_message {
  int control_station_id;
  int light_station_id;
  uint8_t pwmValue;
} struct_message;

struct_message myData;
uint8_t currentPWM = 0; // Track current light value
uint8_t routerMac[] = {0x48, 0xCA, 0x43, 0x2F, 0x6A, 0x24}; // Your router MAC

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  
  if (myData.light_station_id == MY_LIGHT_STATION_ID) {
    Serial.print("Message for Station ");
    Serial.print(MY_LIGHT_STATION_ID);
    Serial.print(" | PWM: ");
    Serial.println(myData.pwmValue);
    
    // Check if this is a request for current value (255) or a new value
    if (myData.pwmValue == 255) {
      // This is a request for current value - send it back
      sendResponse(myData.control_station_id, currentPWM);
      Serial.println("Sent current value to control station");
    } else {
      // This is a new value - update the light
      if (myData.pwmValue != currentPWM) {
        currentPWM = myData.pwmValue;
        analogWrite(pwmPin, currentPWM);
        Serial.print("Light updated to: ");
        Serial.println(currentPWM);
      }
      
      // Always send confirmation back with current value
      sendResponse(myData.control_station_id, currentPWM);
    }
  }
}

void sendResponse(int control_station_id, uint8_t currentPWM) {
  struct_message response;
  response.control_station_id = control_station_id; // Send back to the requesting control station
  response.light_station_id = MY_LIGHT_STATION_ID;
  response.pwmValue = currentPWM;
  
  esp_now_send(routerMac, (uint8_t *) &response, sizeof(response));
}

void setup() {
  Serial.begin(115200);
  pinMode(pwmPin, OUTPUT);
  
  WiFi.mode(WIFI_STA);
  Serial.print("Light Station ID: ");
  Serial.println(MY_LIGHT_STATION_ID);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register receive callback
  esp_now_register_recv_cb(OnDataRecv);
  
  // Add router as peer for sending responses
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, routerMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
  
  Serial.println("Light Station Ready - Bidirectional!");
}

void loop() {
  delay(1000);
}
