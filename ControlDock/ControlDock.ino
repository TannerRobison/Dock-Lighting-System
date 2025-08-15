#include <esp_now.h>
#include <WiFi.h>

const int upperLimit = 100;
const int lowerLimit = 0;
const int encodePin1 = D8;
const int encodePin2 = D9;

int currentMillis = 0; 
int previousMillis = 0;

//These two Variables are used for controlling input reactance
const int pollingInterval = 100;
const int increment = 5;

uint8_t broadcastAddress[] = {0x3C, 0x84, 0x27, 0xC3, 0xD3, 0x64}; //MAC Address of receiver

typedef struct struct_message {
  int userInput;
} struct_message;

struct_message myData;

esp_now_peer_info_t peerInfo; //Stores information about the peer

//Check if package succesfully sent (not delivered)
void OnDataSent (const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


//This function calls when encoder triggers interrupt
void encoderMoved() {
  currentMillis = millis();
  if ((currentMillis - previousMillis) > pollingInterval) {
    
    if (digitalRead(encodePin2) == 1) { //moved left
      if (myData.userInput > lowerLimit) myData.userInput = myData.userInput - (increment*2);
      if (myData.userInput < lowerLimit) myData.userInput = lowerLimit;
    }
    
    if (digitalRead(encodePin2) == 0) { //moved right
      if (myData.userInput < upperLimit) myData.userInput = myData.userInput + increment;
      if (myData.userInput > upperLimit) myData.userInput = upperLimit;
    }
    Serial.print("User Input: ");
    Serial.println(myData.userInput);
    previousMillis = currentMillis;
    return;
  }
  else {
    previousMillis = currentMillis;
    return;
  }
  
}

void setup() {
  Serial.begin(115200);
  pinMode(encodePin1, INPUT_PULLUP);
  pinMode(encodePin2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encodePin1), encoderMoved, FALLING);

  WiFi.mode(WIFI_STA);

  //init ESP_NOW
  if (esp_now_init() != ESP_OK) {
    Serial.print("Error initializing ESP NOW\n");
    return;
  }
  else {
    Serial.print("ESPNOW initialized\n");
    delay(5000);
  }

  esp_now_register_send_cb(OnDataSent);

  //register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  //Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.print("Failed to add peer");
    return;
  }
}

void loop() {
  //sends data
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*) &myData, sizeof(myData));

  //checks if message sent correctly
    //This should be commented out later, maybe not actually
  if (result == ESP_OK) Serial.print("Sent with success\n");
  else Serial.print("Error sending data\n");
  delay(50);
}
