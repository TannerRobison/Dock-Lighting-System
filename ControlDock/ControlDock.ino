#include <esp_now.h>
#include <WiFi.h>
#include <esp_sleep.h>

const int upperLimit = 255;
const int lowerLimit = 0;
const int encodePin1 = D9;
const int encodePin2 = D8;

// Timing variables
unsigned long currentMillis = 0;
unsigned long previousSendMillis = 0;

// Control parameters
const int sendInterval = 50;
const int baseIncrement = 3;
const int accelerationIncrement = 5;
const unsigned long accelerationThreshold = 200;

// Encoder state tracking
int lastEncoded = 0;
unsigned long lastEncoderTime = 0;
int rotationCount = 0;

// Mode select variables
int selectedMode = 1;
int button1 = A1;
int button2 = A2;
int button3 = A3;

// LED pins
int yellowLed = D2;
int orangeLed = D3;
int redLed = D4;

// Sleep variables
unsigned long lastActivityTime = 0;
const unsigned long sleepWarning = 15000; //15 seconds warning
const unsigned long sleepTimeout = 17000; //20 seconds

// Router MAC address
uint8_t routerAddress[] = {0x48, 0xCA, 0x43, 0x2F, 0x6A, 0x24};

// Control station ID - MAKE UNIQUE FOR EACH CONTROL STATION
const int CONTROL_STATION_ID = 1;

typedef struct struct_message {
  int control_station_id;
  int light_station_id;
  uint8_t pwmValue;
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

// Track current values for each light station
uint8_t lightStationValues[4] = {0, 0, 0, 0}; // Index 0 unused, 1-3 for light stations

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("✓ Delivery Success");
  } else {
    Serial.println("✗ Delivery Fail");
  }
}

// Callback when data is received from light stations
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  struct_message receivedData;
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  
  // Update the stored value for this light station
  if (receivedData.light_station_id >= 1 && receivedData.light_station_id <= 3) {
    lightStationValues[receivedData.light_station_id] = receivedData.pwmValue;
    
    // If this is for our currently selected mode, update display
    if (receivedData.light_station_id == selectedMode) {
      myData.pwmValue = receivedData.pwmValue;
      Serial.print("Updated current value for station ");
      Serial.print(selectedMode);
      Serial.print(": ");
      Serial.println(myData.pwmValue);
    }
  }
}

// Encoder reading
void readEncoder() {
  int MSB = digitalRead(encodePin1);
  int LSB = digitalRead(encodePin2);
  
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;
  
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    updateBrightness(1);
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    updateBrightness(-1);
  }
  
  lastEncoded = encoded;
}

void updateBrightness(int direction) {
  //currentMillis = millis();
  lastActivityTime = millis();
  
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
  int newValue = myData.pwmValue + (direction * increment);
  newValue = constrain(newValue, lowerLimit, upperLimit);
  
  // Only update if value changed
  if (newValue != myData.pwmValue) {
    myData.pwmValue = newValue;
    
    // Update stored value for this light station
    lightStationValues[selectedMode] = newValue;
    
    Serial.print("User Input: ");
    Serial.print(myData.pwmValue);
    Serial.print(" | Light Station: ");
    Serial.println(myData.light_station_id);
    
    // Send the update
    sendData();
  }
}

void changeMode(){
  int oldMode = selectedMode;
  
  // Read buttons and set mode
  if (digitalRead(button1)) {
    selectedMode = 1;
  } else if (digitalRead(button2)) {
    selectedMode = 2;
  } else if (digitalRead(button3)) {
    selectedMode = 3;
  }

  // Update if mode changed
  if (oldMode != selectedMode) {
    lastActivityTime = millis();
    myData.light_station_id = selectedMode;
    
    // Update LED indicators
    digitalWrite(yellowLed, (selectedMode == 3));
    digitalWrite(orangeLed, (selectedMode == 2));
    digitalWrite(redLed, (selectedMode == 1));
    
    // Set current value to the stored value for this light station
    myData.pwmValue = lightStationValues[selectedMode];
    
    Serial.print("Mode changed to: ");
    Serial.print(selectedMode);
    Serial.print(" | Current value: ");
    Serial.println(myData.pwmValue);
    
    // Request current value from light station to ensure we're in sync
    requestCurrentValue();
  }
}

void requestCurrentValue() {
  // Send a special request to get the current value
  struct_message requestData;
  requestData.control_station_id = CONTROL_STATION_ID;
  requestData.light_station_id = selectedMode;
  requestData.pwmValue = 255; // Special value indicating "request current state"
  
  esp_err_t result = esp_now_send(routerAddress, (uint8_t*) &requestData, sizeof(requestData));
  
  if (result == ESP_OK) {
    Serial.print("Requested current value from light station ");
    Serial.println(selectedMode);
  } else {
    Serial.println("Error requesting current value");
  }
}

void sendData() {
  esp_err_t result = esp_now_send(routerAddress, (uint8_t*) &myData, sizeof(myData));
  
  if (result == ESP_OK) {
    Serial.print("Sent: Station ");
    Serial.print(myData.light_station_id);
    Serial.print(" | PWM: ");
    Serial.println(myData.pwmValue);
  } else {
    Serial.println("Error sending data");
  }
  
  previousSendMillis = currentMillis;
}

void checkSleep(){
  //Serial.println("Checking if needs to sleep");
  Serial.println(currentMillis - lastActivityTime);
  if ((currentMillis - lastActivityTime) > sleepTimeout) {
    Serial.println("Entering deep sleep, goodnight");

    //wake up on encoder movement or button press
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_18, LOW);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_17, LOW);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_2, HIGH);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_3, HIGH);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, HIGH);

    //turn off Led's
    digitalWrite(redLed, LOW);
    digitalWrite(yellowLed, LOW);
    digitalWrite(orangeLed,LOW);
    
    //goodnight
    esp_deep_sleep_start();
  }

  //5 second warning
  else if ((currentMillis - lastActivityTime) > sleepWarning) {
    //flash lights on
    digitalWrite(redLed, HIGH);
    digitalWrite(orangeLed, HIGH);
    digitalWrite(yellowLed, HIGH);
    delay(50);

    //flash lights off
    digitalWrite(redLed, LOW);
    digitalWrite(orangeLed, LOW);
    digitalWrite(yellowLed, LOW);
    delay(50);
  }
}

void setup() {
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(encodePin1, INPUT_PULLUP);
  pinMode(encodePin2, INPUT_PULLUP);
  pinMode(button1, INPUT_PULLDOWN);
  pinMode(button2, INPUT_PULLDOWN);
  pinMode(button3, INPUT_PULLDOWN);
  
  // LED pins
  pinMode(yellowLed, OUTPUT);
  pinMode(orangeLed, OUTPUT);
  pinMode(redLed, OUTPUT);

  // Encoder interrupts
  attachInterrupt(digitalPinToInterrupt(encodePin1), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encodePin2), readEncoder, CHANGE);

  // Initialize WiFi
  WiFi.mode(WIFI_STA);

  // Print MAC address
  Serial.print("Control Station MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Control Station ID: ");
  Serial.println(CONTROL_STATION_ID);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callbacks
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Register router as peer
  memcpy(peerInfo.peer_addr, routerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add router as peer");
    return;
  }

  // Initialize data structure
  myData.control_station_id = CONTROL_STATION_ID;
  myData.light_station_id = selectedMode;
  myData.pwmValue = 0;

  // Initialize light station values
  lightStationValues[1] = 0;
  lightStationValues[2] = 0;
  lightStationValues[3] = 0;

  // Set initial LED state
  digitalWrite(redLed, HIGH);

  Serial.println("Control Station Ready - Bidirectional!");
  Serial.println("Use buttons 1-3 to select light station, encoder to adjust brightness");
}

void loop() {
  currentMillis = millis();
  
  // Check for mode changes
  changeMode();
  
  //check sleep
  checkSleep();
  
  delay(1);
}
