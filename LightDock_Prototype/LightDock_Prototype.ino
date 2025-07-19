#include <math.h>

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
const int pollingInterval = 10;
const int increment = 5;

void encoderMoved() {
  ///This Function is called when the encoder is moved
  currentMillis = millis();
  if ((currentMillis - previousMillis) > pollingInterval) { 
    if (digitalRead(encodePin2) == 1) { //moved left
      if (userInput > 0) userInput = userInput - (increment*2);
      if (userInput < 0) userInput = 0;
    }
    
    if (digitalRead(encodePin2) == 0) { //moved right
      if (userInput < upperLimit) userInput = userInput + increment;
      if (userInput > upperLimit) userInput = upperLimit;
    }

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
 pinMode(encodePin1, INPUT_PULLUP);
 pinMode(encodePin2, INPUT_PULLUP);

 attachInterrupt(digitalPinToInterrupt(encodePin1), encoderMoved, FALLING);
}

void loop() {
  
}
