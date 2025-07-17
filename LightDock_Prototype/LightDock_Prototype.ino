#include <math.h>

int userInput = 0;
int scaledInput = 0;
int lightValue = 0; 
const int pwmPin = 4;
const int encodePin1 = 8;
const int encodePin2 = 9;
const int upperLimit = 100;
//These Two variables are for debouncing
int currentMillis = 0; 
int previousMillis = 0;
const int pollingInterval = 50;
const int increment = 4;

void setup() {
 pinMode(pwmPin, OUTPUT);
 
 Serial.begin(115200);
 Serial.print("Booting up...");

 pinMode(encodePin1, INPUT);
 pinMode(encodePin2, INPUT);
}

void loop() {
  //determines user input
  
  currentMillis = millis();
  
  if ((currentMillis - previousMillis) >= pollingInterval) {
    if (digitalRead(encodePin1) == 0 && digitalRead(encodePin2) == 1) {
      if (userInput > 0) userInput = userInput - increment;
      //if (userInput > upperLimit) userInput = upperLimit;
      if (userInput < 0) userInput = 0;
      }
    if (digitalRead(encodePin1) == 1 && digitalRead(encodePin2) == 0) {
      if (userInput < upperLimit) userInput = userInput + increment;
      if (userInput > upperLimit) userInput = upperLimit;
    }
    previousMillis = millis();
  }
  //puts input on an exponential scale
  if (userInput > 0) scaledInput = pow(3, userInput / 20);
  if (userInput == 0) scaledInput = 0;

  //maps scaled Input range to pwm signal values
  lightValue = map(scaledInput, 0,  243, 0, 255); //243 because 3^(x/20) = 243
  analogWrite(pwmPin, lightValue); //sends signal to LED bulb
  
  //Serial monitoring
  Serial.print("\nuser input: ");
  Serial.print(userInput);

  Serial.print("  Scaled input: ");
  Serial.print(scaledInput);
  
  Serial.print("  light Value: ");
  Serial.print(lightValue);
}
