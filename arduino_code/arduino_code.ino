#include <Servo.h>

// Motor control pins
const int MOTOR1_PIN = 8;   // Left motor
const int MOTOR2_PIN = 9;  // Right motor
const int MOTOR3_PIN = 10;  // Front motor
const int MOTOR4_PIN = 11;   // Back motor

// PWM safe range
const int PWM_MIN = 1100;
const int PWM_MAX = 1900;
const int PWM_STOP = 1500;

Servo motor1, motor2, motor3, motor4;

String inputString = "";
boolean stringComplete = false;

void setup() {
  Serial.begin(115200);
  motor1.attach(MOTOR1_PIN);
  motor2.attach(MOTOR2_PIN);
  motor3.attach(MOTOR3_PIN);
  motor4.attach(MOTOR4_PIN);
  motor1.writeMicroseconds(PWM_STOP);
  motor2.writeMicroseconds(PWM_STOP);
  motor3.writeMicroseconds(PWM_STOP);
  motor4.writeMicroseconds(PWM_STOP);
  delay(3000); // ESC initialization
}

void loop() {
  if (stringComplete) {
    if (inputString.startsWith("<[") && inputString.endsWith("]>")) {
      int pwmValues[4];
      int startIdx = 2;
      for (int i = 0; i < 4; i++) {
        int endIdx = inputString.indexOf("]", startIdx);
        if (endIdx != -1) {
          pwmValues[i] = inputString.substring(startIdx, endIdx).toInt();
          startIdx = endIdx + 2;
        }
      }
      for (int i = 0; i < 4; i++) {
        pwmValues[i] = constrain(pwmValues[i], PWM_MIN, PWM_MAX);
      }
      motor1.writeMicroseconds(pwmValues[0]);
      motor2.writeMicroseconds(pwmValues[1]);
      motor3.writeMicroseconds(pwmValues[2]);
      motor4.writeMicroseconds(pwmValues[3]);
      Serial.print("<[");
      for (int i = 0; i < 4; i++) {
        Serial.print(pwmValues[i]);
        if (i < 3) Serial.print("][");
      }
      Serial.println("]>");
      inputString = "";
      stringComplete = false;
    }
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '>') {
      stringComplete = true;
    }
  }
} 
