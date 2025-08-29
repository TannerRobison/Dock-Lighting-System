#include <esp_now.h>
#include <WiFi.h>
#include <math.h>

const int pwmPin = D4;

typedef struct struct_message {
  int userInput;
} struct_message;

struct_message myData;

//Callback function executed when data is recieved
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  encoderMoved (myData.userInput);
}

void encoderMoved (int inputData){
  //puts input on exponential scale
  double scaledInput = pow(3, (double)inputData / 20);
  
  //Maps scaled input range to pwm signal range
  float lightValue = map(scaledInput, 0,  243, 0, 255); //243 because 3^(x/20) = 243

  if (inputData == 0) lightValue = 0;
  analogWrite(pwmPin, lightValue);

  //Serial monitoring
  Serial.print("\nUser Input: ");
  Serial.println(inputData);
  Serial.print ("\nScaled Input: ");
  Serial.println(scaledInput);
  Serial.print("\nLight Value: ");
  Serial.println(lightValue);
  
}

void setup() {
  Serial.begin(115200);
  pinMode(pwmPin, OUTPUT);
  
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
