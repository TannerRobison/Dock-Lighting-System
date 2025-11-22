#include <esp_now.h>
#include <WiFi.h>

const int upperLimit = 100;
const int lowerLimit = 0;
const int encodePin1 = D9;
const int encodePin2 = D8;

// Improved timing variables
unsigned long currentMillis = 0;
unsigned long previousEncoderMillis = 0;
unsigned long previousSendMillis = 0;

// Smoother control parameters
const int encoderPollingInterval = 2; // Much shorter for responsiveness
const int sendInterval = 50; // Control how often we send data
const int baseIncrement = 1;
const int accelerationIncrement = 5;
const unsigned long accelerationThreshold = 200; // ms for acceleration

// Encoder state tracking
int encoderPos = 0;
int lastEncoded = 0;
unsigned long lastEncoderTime = 0;
int rotationCount = 0; // For acceleration

//mode select variables
int selectedMode = 0;
int button1 = A1;
int button2 = A2;
int button3 = A3;
//LED pins
int yellowLed = D2;
int orangeLed = D3;
int redLed = D4;

uint8_t broadcastAddress[] = {0x3C, 0x84, 0x27, 0xC3, 0xD3, 0x64};

typedef struct struct_message {
  int userInput;
  bool valueChanged = false;
} struct_message;

struct_message myData;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    myData.valueChanged = false; // Reset flag on successful send
  }
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Improved encoder reading using state machine
void readEncoder() {
  int MSB = digitalRead(encodePin1);
  int LSB = digitalRead(encodePin2);
  
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;
  
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    // Clockwise
    updateBrightness(1);
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    // Counter-clockwise
    updateBrightness(-1);
  }
  
  lastEncoded = encoded;
}

void updateBrightness(int direction) {
  currentMillis = millis();
  
  // Calculate acceleration
  int increment = baseIncrement;
  if (currentMillis - lastEncoderTime < accelerationThreshold) {
    rotationCount++;
    if (rotationCount > 3) {
      increment = accelerationIncrement;
    }
  } else {
    rotationCount = 0;
  }
  
  lastEncoderTime = currentMillis;
  
  // Apply change
  int newValue = myData.userInput + (direction * increment);
  
  // Constrain value
  newValue = constrain(newValue, lowerLimit, upperLimit);
  
  // Only update if value changed
  if (newValue != myData.userInput) {
    myData.userInput = newValue;
    myData.valueChanged = true;
    
    Serial.print("User Input: ");
    Serial.println(myData.userInput);
  }
}

void changeMode(){
  int oldMode = selectedMode;
  if (digitalRead(button1)) selectedMode = 1;
  if (digitalRead(button2)) selectedMode = 2;
  if (digitalRead(button3)) selectedMode = 3;

  if (oldMode != selectedMode) {
    switch(selectedMode) {
      case 1:
        digitalWrite(yellowLed, HIGH);
        digitalWrite(orangeLed, LOW);
        digitalWrite(redLed, LOW);
        break;
      case 2:
        digitalWrite(orangeLed, HIGH);
        digitalWrite(yellowLed, LOW);
        digitalWrite(redLed, LOW);
        break;
      case 3:
        digitalWrite(redLed, HIGH);
        digitalWrite(yellowLed, LOW);
        digitalWrite(orangeLed, LOW);
        break;
    }
  }
  //for testing purposes
  digitalWrite(redLed, HIGH);
  Serial.print("red led should be glowing");
}

void setup() {
  Serial.begin(115200);
  pinMode(encodePin1, INPUT_PULLUP);
  pinMode(encodePin2, INPUT_PULLUP);
  pinMode(button1, INPUT_PULLDOWN);
  pinMode(button2, INPUT_PULLDOWN);
  pinMode(button3, INPUT_PULLDOWN);

  // Use both pins for interrupts for more reliable reading
  attachInterrupt(digitalPinToInterrupt(encodePin1), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encodePin2), readEncoder, CHANGE);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.print("Error initializing ESP NOW\n");
    return;
  } else {
    Serial.print("ESPNOW initialized\n");
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.print("Failed to add peer");
    return;
  }
}

void loop() {
  currentMillis = millis();
  
  // Only send data when value has changed and at controlled interval
  if (myData.valueChanged && (currentMillis - previousSendMillis >= sendInterval)) {
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*) &myData, sizeof(myData));
    
    if (result == ESP_OK) {
      Serial.print("Sent with success: ");
      Serial.println(myData.userInput);
    } else {
      Serial.print("Error sending data\n");
    }
    
    previousSendMillis = currentMillis;
  }
  
  delay(1); // Small delay to prevent overwhelming the processor
}
