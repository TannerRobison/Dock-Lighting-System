#include <esp_now.h>
#include <WiFi.h>
#include <math.h>

typedef struct struct_message {
  int userInput;
} struct_message;

struct_message myData;

//Callback function executed when data is recieved
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Recieving: ");
  Serial.println(myData.userInput);
}

void encoderMoved (int inputData){
  int scaledInput = pow(3, inputData / 20);
  int lightValue = map(scaledInput, 0,  243, 0, 255); //243 because 3^(x/20) = 243
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  //init ESP_NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop() {

  
}
