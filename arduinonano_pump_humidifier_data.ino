// Define pins
#define IR_LEFT A0
#define IR_RIGHT A1
#define US_TRIG 2
#define US_ECHO 3
#define SERVO_PIN 9

// Define constants
#define MAX_DISTANCE 100 // Maximum distance for ultrasonic sensor (cm)
#define TURN_SPEED 1500 // Servo speed for turning (adjust as needed)
#define FORWARD_SPEED 1600 // Servo speed for forward movement (adjust as needed)
#define STOP_SPEED 1500 // Servo speed for stop (center position)
#define IR_THRESHOLD 500 // Adjust based on your sensor readings

#include <Servo.h>

Servo servo;

void setup() {
  Serial.begin(9600);
  servo.attach(SERVO_PIN);
  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);
}

long getDistance() {
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);

  long duration = pulseIn(US_ECHO, HIGH);
  return duration / 29 / 2; // Calculate distance in cm
}

void loop() {
  int irLeft = analogRead(IR_LEFT);
  int irRight = analogRead(IR_RIGHT);
  long distance = getDistance();

  Serial.print("IR Left: ");
  Serial.print(irLeft);
  Serial.print(" IR Right: ");
  Serial.print(irRight);
  Serial.print(" Distance: ");
  Serial.println(distance);

  if (distance < 20) { // Stop if too close to an obstacle
    servo.writeMicroseconds(STOP_SPEED);
     Serial.println("Obstacle too Close");
    delay(50);
    return;
  }
  
  if (irLeft > IR_THRESHOLD && irRight > IR_THRESHOLD) {
    // Both sensors detect the person, move forward
    if (distance < MAX_DISTANCE) {
         servo.writeMicroseconds(FORWARD_SPEED);
          Serial.println("Moving Forward");
    } else {
        servo.writeMicroseconds(STOP_SPEED);
         Serial.println("Target too far");
    }
  } else if (irLeft > IR_THRESHOLD) {
    // Person is on the left, turn left
    servo.writeMicroseconds(TURN_SPEED - 100); // Adjust the value for turning speed
    Serial.println("Turning Left");
  } else if (irRight > IR_THRESHOLD) {
    // Person is on the right, turn right
    servo.writeMicroseconds(TURN_SPEED + 100); // Adjust the value for turning speed
     Serial.println("Turning Right");
  } else {
    // No person detected, stop
    servo.writeMicroseconds(STOP_SPEED);
     Serial.println("No Target Found");
  }
  delay(50); // Small delay for stability

}