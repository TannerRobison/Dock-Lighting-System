#include <esp_now.h>
#include <WiFi.h>

const int upperLimit = 255;  // Changed to 255 for PWM
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
const int baseIncrement = 3;
const int accelerationIncrement = 5;
const unsigned long accelerationThreshold = 200; // ms for acceleration

// Encoder state tracking
int encoderPos = 0;
int lastEncoded = 0;
unsigned long lastEncoderTime = 0;
int rotationCount = 0; // For acceleration

// Mode select variables
int selectedMode = 1;  // Start with mode 1 (controls light station 1)
int button1 = A1; //mode 1
int button2 = A2; //mode 2
int button3 = A3; //mode 3

// LED pins
int yellowLed = D2;
int orangeLed = D3;
int redLed = D4;

// Router MAC address (your central hub)
uint8_t routerAddress[] = {0x48, 0xCA, 0x43, 0x2F, 0x6A, 0x24}; // Router MAC

// Control station ID - MAKE THIS UNIQUE FOR EACH CONTROL STATION
const int CONTROL_STATION_ID = 1; // Change this for each control station (1, 2, 3, etc.)

typedef struct struct_message {
  int control_station_id;
  int light_station_id;
  uint8_t pwmValue;
  bool valueChanged = false;
} struct_message;

struct_message myData;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    myData.valueChanged = false; // Reset flag on successful send
    Serial.println("✓ Delivery Success");
  } else {
    Serial.println("✗ Delivery Fail");
  }
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
  
  // Apply change - scale to 0-255 for PWM
  int newValue = myData.pwmValue + (direction * increment);
  
  // Constrain value
  newValue = constrain(newValue, lowerLimit, upperLimit);
  
  // Only update if value changed
  if (newValue != myData.pwmValue) {
    myData.pwmValue = newValue;
    myData.valueChanged = true;
    
    Serial.print("User Input: ");
    Serial.print(myData.pwmValue);
    Serial.print(" | Light Station: ");
    Serial.println(myData.light_station_id);
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

  // Update LEDs and light station target only if mode changed
  if (oldMode != selectedMode) {
    myData.light_station_id = selectedMode; // Set target light station
    myData.valueChanged = true; // Force send to update the light station
    
    // Update LED indicators
    digitalWrite(yellowLed, (selectedMode == 3));
    digitalWrite(orangeLed, (selectedMode == 2));
    digitalWrite(redLed, (selectedMode == 1));
    
    Serial.print("Mode changed to: ");
    Serial.print(selectedMode);
    Serial.print(" | Controlling Light Station: ");
    Serial.println(selectedMode);
    
    // Send immediate update
    sendData();
  }
}

void sendData() {
  if (myData.valueChanged) {
    esp_err_t result = esp_now_send(routerAddress, (uint8_t*) &myData, sizeof(myData));
    
    if (result == ESP_OK) {
      Serial.print("Sent: Station ");
      Serial.print(myData.light_station_id);
      Serial.print(" | PWM: ");
      Serial.println(myData.pwmValue);
    } else {
      Serial.println("Error sending data");
      // Don't reset valueChanged on error - retry next loop
    }
    
    previousSendMillis = currentMillis;
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

  // Use both pins for interrupts for more reliable reading
  attachInterrupt(digitalPinToInterrupt(encodePin1), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encodePin2), readEncoder, CHANGE);

  // Initialize WiFi
  WiFi.mode(WIFI_STA);

  // Print MAC address (useful for router configuration)
  Serial.print("Control Station MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Control Station ID: ");
  Serial.println(CONTROL_STATION_ID);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

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
  myData.light_station_id = selectedMode;  // Start with controlling light station 1
  myData.pwmValue = 0;  // Start at 0 brightness
  myData.valueChanged = false;

  // Set initial LED state
  digitalWrite(redLed, HIGH);  // Mode 1 active initially

  Serial.println("Control Station Ready!");
  Serial.println("Use buttons 1-3 to select light station, encoder to adjust brightness");
}

void loop() {
  currentMillis = millis();
  
  // Check for mode changes
  changeMode();
  
  // Send data when value has changed and at controlled interval
  if (myData.valueChanged && (currentMillis - previousSendMillis >= sendInterval)) {
    sendData();
  }
  
  delay(1); // Small delay to prevent overwhelming the processor
}
