#include <esp_now.h>
#include <WiFi.h>

const int upperLimit = 100;
const int lowerLimit = 0;
const int encodePin1 = D8;
const int encodePin2 = D9;

int currentMillis = 0; 
int previousMillis = 0;

//These two Variables are used for controlling input reactance
const int pollingInterval = 10;
const int increment = 5;

//This function calls when encoder triggers interrupt
void encoderMoved() {
  currentMillis = millis();
  if ((currentMillis - previousMillis) > pollingInterval) {
    
    if (digitalRead(encodePin2) == 1) { //moved left
      if (userInput > lowerLimit) myData.userInput = myData.userInput - (increment*2);
      if (userInput < lowerLimit) myData.userInput = 0;
    }
    
    if (digitalRead(encodePin2) == 0) { //moved right
      if (userInput < upperLimit) myData.userInput = myData.userInput + increment;
      if (userInput > upperLimit) myData.userInput = upperLimit;
    }
  }

  
}


uint8_t broadcastAddress[] = {0x3C, 0x84, 0x27, 0xC3, 0xD3, 0x64};

typedef struct struct_message {
  int userInput;
} struct_message

struct_message myData;

esp_now_peer_info_t peerInfo; //Stores information about the peer

void OnDataSent (const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  //init ESP_NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP NOW");
    return;
  }

  pinMode(encodePin1, INPUT_PULLUP);
  pinMode(encodePin2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encodePin1), encoderMoved, FALLING);
}

void loop() {
  //sends data
  esp_err_t result = esp_now_send(broadcastAdress, (uint8_t*) &myData, sizeof(myData));

  //checks if message sent correctly
    //This should be commented out later
  if (result == ESP_OK) Serial.print("Sent with success");
  else Serial.print("Error sending data");
  delay(50);
}
