
// ***************************************************************
// This Code controls a DC motor with a potentiometer and pwm
// The center of the pot is zero rotation and negative values spin the motor one way and positive values spin the dc motor the other way
// Provides full power to motor in extreme potentiometer positions


#define enA 9
#define in1 6
#define in2 7
#define button 4

int rotDirection = 0;
int pressed = false;


void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(button, INPUT);
  // Set initial rotation direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  Serial.begin(115200);
}

void loop() {
  
  int potValue = analogRead(A0); // Read potentiometer value
  
  int pwmOutput = map(potValue, 0, 1023, -255, 255); // Map the potentiometer value from 0 to 255
  
  // If negative 
  if (pwmOutput < 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  // If button is pressed - change rotation direction
  if (pwmOutput > 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }  
  
  pwmOutput = abs(pwmOutput); // Once direction is set output the magnitude of the pwm signal

  analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin
  // // Read button - Debounce
  // if (digitalRead(button) == true) {
  //   pressed = !pressed;
  // }
  // while (digitalRead(button) == true);
  // delay(20);

  
}