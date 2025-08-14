#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddress[] = {0x3C, 0x84, 0x27, 0xC3, 0xD3, 0x64}; //replace with reciever MAC address

typedef struct struct_message {
  char a[32];
  int b;
  float c;
  bool d;
} struct_message;


//create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo; //This variable stores information about the peer

void OnDataSent (const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);

  //sets device as a WiFi station
  WiFi.mode(WIFI_STA);

  //init ESP_NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  //once ESP_NOW is succesfully init, register for send CB to get status of Transmitted packet
  esp_now_register_send_cb(OnDataSent);

  //register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  //add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  //set values to send
  strcpy(myData.a, "This is a char");
  myData.b = random(1,20);
  myData.c = 1.2;
  myData.d = false;

  //send message
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

  if (result == ESP_OK) Serial.print("Sent with success");
  else Serial.print("Error sending data");
  delay(2000);

}
