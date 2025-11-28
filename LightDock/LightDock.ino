#include <esp_now.h>
#include <WiFi.h>
#include <math.h>

const int pwmPin = D4;

// ADD THIS - Set a unique ID for each light station
const int MY_LIGHT_STATION_ID = 1; // Change this to 2, 3, etc. for other light stations

typedef struct struct_message {
  int control_station_id;
  int light_station_id;
  uint8_t pwmValue;
  bool valueChanged = false;
} struct_message;

struct_message myData;

//Callback function executed when data is recieved
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  
  // ADD THIS FILTER - Only process if this message is for THIS light station
  if (myData.light_station_id == MY_LIGHT_STATION_ID) {
    Serial.print("Message for ME! Station ");
    Serial.println(MY_LIGHT_STATION_ID);
    encoderMoved(myData.pwmValue);
  } else {
    Serial.print("Ignoring message for station ");
    Serial.println(myData.light_station_id);
  }
}

void encoderMoved (int inputData){
  int lightValue = inputData;
  if (inputData == 0) lightValue = 0;
  analogWrite(pwmPin, lightValue);

  Serial.print("Light Value: ");
  Serial.println(lightValue);
}

void setup() {
  Serial.begin(115200);
  pinMode(pwmPin, OUTPUT);
  
  WiFi.mode(WIFI_STA);

  // Print this light station's ID
  Serial.print("Light Station ID: ");
  Serial.println(MY_LIGHT_STATION_ID);

  //init ESP_NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  Serial.println("Light Station Ready!");
}

void loop() {
  // Empty loop
}
