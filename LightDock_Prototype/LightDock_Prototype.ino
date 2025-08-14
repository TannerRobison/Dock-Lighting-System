#include <math.h>
#include <esp_now.h>
#include <WiFi.h>

int userInput = 0;
int scaledInput = 0;
int lightValue = 0; 
const int pwmPin = D4;
const int encodePin1 = D8;
const int encodePin2 = D9;
const int upperLimit = 100;

//These Two variables are for debouncing
int currentMillis = 0; 
int previousMillis = 0;

//These two Variables are used for controlling input reactance
const int pollingInterval = 10;
const int increment = 5;

void encoderMoved() {
  ///This Function is called when data is received

    //puts input on an exponential scale
    scaledInput = pow(3, userInput / 20);

    //maps scaled Input range to pwm signal values
    lightValue = map(scaledInput, 0,  243, 0, 255); //243 because 3^(x/20) = 243
    //if (scaledInput == 243) lightValue = 255;
    //else lightValue = (int)scaledInput;
    
    if (userInput == 0) lightValue = 0;
    analogWrite(pwmPin, lightValue); //sends signal to LED bulb
    
    //Serial monitoring
    Serial.print("\nuser input: ");
    Serial.print(userInput);
  
    Serial.print("  Scaled input: ");
    Serial.print(scaledInput);
    
    Serial.print("  light Value: ");
    Serial.print(lightValue);

    previousMillis = currentMillis;
  }
}

void setup() {
 Serial.begin(115200);
 Serial.print("Booting up...");

 pinMode(pwmPin, OUTPUT);
}

void loop() {
  
}
