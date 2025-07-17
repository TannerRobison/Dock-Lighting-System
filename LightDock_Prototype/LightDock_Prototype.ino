#include <math.h>

//Variables
int userInput = 0;
int scaledInput = 0;
int lightValue = 0; 
int pwmPin = 4;
int encodePin1 = 8;
int encodePin2 = 9;
int upperLimit = 100;

void setup() {
 pinMode(pwmPin, OUTPUT);
 
 Serial.begin(115200);
 Serial.print("Booting up...");

 pinMode(encodePin1, INPUT);
 pinMode(encodePin2, INPUT);
}

void loop() {
  //determines user input
  if (digitalRead(encodePin1) == 0 && digitalRead(encodePin2) == 1) {
    if (userInput > 0) userInput = userInput - 4;
    //if (userInput > upperLimit) userInput = upperLimit;
    if (userInput < 0) userInput = 0;
    
  }
  if (digitalRead(encodePin1) == 1 && digitalRead(encodePin2) == 0) {
    if (userInput < upperLimit) userInput = userInput + 4;
    if (userInput > upperLimit) userInput = upperLimit;
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
