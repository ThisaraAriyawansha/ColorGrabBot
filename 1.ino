#include <Wire.h>
#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

const int S0 = 12;
const int S1 = 13;
const int S2 = 14;
const int S3 = 15;
const int sensorOut = 16;

// Define color thresholds (adjust these values based on your calibration)
const int redThreshold = 220;   // Adjust as needed
const int greenThreshold = 220; // Adjust as needed
const int blueThreshold = 220;  // Adjust as needed
// Motor 1
int motor1PWM = 9;   // PWM pin for motor 1 speed control
int motor1DirA = 8;   // Direction pin A for motor 1
int motor1DirB = 7;   // Direction pin B for motor 1

// Motor 2
int motor2PWM = 6;   // PWM pin for motor 2 speed control
int motor2DirA = 5;   // Direction pin A for motor 2
int motor2DirB = 4;   // Direction pin B for motor 2

// IR Sensor 1
int irSensor1 = 2;   // Left IR sensor input pin
// IR Sensor 2
int irSensor2 = 3;   // Right IR sensor input pin

// Ultrasonic sensor
int trigPin = 10;     // Trigger pin of the ultrasonic sensor
int echoPin = 11;     // Echo pin of the ultrasonic sensor

void setup() {

  servo1.attach(17);
  servo2.attach(18);
  servo3.attach(19);
  servo4.attach(20);

   pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  Serial.begin(9600);

  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  pinMode(motor1PWM, OUTPUT);
  pinMode(motor1DirA, OUTPUT);
  pinMode(motor1DirB, OUTPUT);

  pinMode(motor2PWM, OUTPUT);
  pinMode(motor2DirA, OUTPUT);
  pinMode(motor2DirB, OUTPUT);

  pinMode(irSensor1, INPUT);
  pinMode(irSensor2, INPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.begin(9600);
}

void moveMotorsForward() {
  digitalWrite(motor1DirA, HIGH);
  digitalWrite(motor1DirB, LOW);
  analogWrite(motor1PWM, 150);

  digitalWrite(motor2DirA, HIGH);
  digitalWrite(motor2DirB, LOW);
  analogWrite(motor2PWM, 188);
}

void turnLeft() {
  digitalWrite(motor1DirA, LOW);
  digitalWrite(motor1DirB, HIGH);
  analogWrite(motor1PWM, 150);

  digitalWrite(motor2DirA, HIGH);
  digitalWrite(motor2DirB, LOW);
  analogWrite(motor2PWM, 150);
}

void turnRight() {
  digitalWrite(motor1DirA, HIGH);
  digitalWrite(motor1DirB, LOW);
  analogWrite(motor1PWM, 150);

  digitalWrite(motor2DirA, LOW);
  digitalWrite(motor2DirB, HIGH);
  analogWrite(motor2PWM, 150);
}

void stopMotors() {
  digitalWrite(motor1DirA, LOW);
  digitalWrite(motor1DirB, LOW);
  analogWrite(motor1PWM, 0);

  digitalWrite(motor2DirA, LOW);
  digitalWrite(motor2DirB, LOW);
  analogWrite(motor2PWM, 0);
  
  int red = readColor(2);
  int green = readColor(3);
  int blue = readColor(1);

  // Check for Red color
  if (red > redThreshold && green < greenThreshold && blue < blueThreshold) {
    Serial.println("Detected color: Red");
    rotateServosClockwise();
   delay(1000);
  } 
  // Check for Green color
  else if (red < redThreshold && green > greenThreshold && blue < blueThreshold) {
    Serial.println("Detected color: Green");
  } 
  // Check for Blue color
  else if (red < redThreshold && green < greenThreshold && blue > blueThreshold) {
    Serial.println("Detected color: Blue");
     rotateServosCounterClockwise();
  delay(1000);
  } 
  // Unknown color
  else {
    Serial.println("Unknown color");
  }

  delay(1000);
}

int readColor(int color) {
  if (color == 1) {
    digitalWrite(S2, LOW);
    digitalWrite(S3, LOW);
  } else if (color == 2) {
    digitalWrite(S2, HIGH);
    digitalWrite(S3, HIGH);
  } else if (color == 3) {
    digitalWrite(S2, LOW);
    digitalWrite(S3, HIGH);
  }

  return pulseIn(sensorOut, LOW);

}

int getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH) / 58.2;  // Convert time to distance in cm
}

void loop() {
  int ir1Value = digitalRead(irSensor1);
  int ir2Value = digitalRead(irSensor2);

  Serial.print("IR Sensor 1: ");
  Serial.println(ir1Value);
  Serial.print("IR Sensor 2: ");
  Serial.println(ir2Value);

  int distance = getDistance();

  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance > 20) {  // Adjust this distance based on your needs
    if (ir1Value == LOW && ir2Value == HIGH) {
      // Move forward if the left sensor detects the line
       turnRight();
    } else if (ir1Value == HIGH && ir2Value == LOW) {
      // Turn left if the right sensor detects the line
      turnLeft();
    } else if (ir1Value == LOW && ir2Value == LOW) {
      // Both sensors off the line, stop the motors
      moveMotorsForward();
      //stopMotors();
    } else {
      // Stop if no specific condition is met
      stopMotors();
    }
  } else {
    // Stop if the obstacle is too close
    stopMotors();
  }

  // Adjust delay as needed
  delay(10);
}
void rotateServosClockwise() {
  rotateServo(servo1, 90);
  rotateServo(servo2, 90);
  //rotateServo(servo3, -90);
  rotateServo(servo4, 90);
}

void rotateServosCounterClockwise() {
 rotateServo(servo4, -90);
  //rotateServo(servo3, 90);
  rotateServo(servo2, -90);
  rotateServo(servo1, -90);
}

void rotateServo(Servo &servo, int angle) {
  int currentAngle = servo.read();
  int targetAngle = currentAngle + angle;
  
  for (int i = currentAngle; i != targetAngle; i += (angle > 0) ? 1 : -1) {
    servo.write(i);
    delay(15); // Adjust delay as needed for your desired servo speed
  }
}
