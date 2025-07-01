// Include the ESP32Servo library
// You might need to install this library through the Arduino IDE's Library Manager.
// Search for "ESP32Servo" and install it.
#include <ESP32Servo.h>

// Define the pins connected to the servo signal wires
// Changed to pins 12 and 13 as requested, and added pins 14 and 27 for two more servos.
const int servoPin1 = 12; // GPIO pin for the first servo
const int servoPin2 = 13; // GPIO pin for the second servo
const int servoPin3 = 14; // GPIO pin for the third servo
const int servoPin4 = 27; // GPIO pin for the fourth servo

// Create Servo objects for each servo
Servo servo1;
Servo servo2;
Servo servo3; // New Servo object for the third servo
Servo servo4; // New Servo object for the fourth servo

// single motor control function
// Corrected function definition: specify types for p1 and p2
void motor_control(int p1, int p2) { // Changed return type to void as it doesn't return anything
  // --- Movement: From p1 to p2 degrees ---
  if (p1 > p2) {
    for (int pos = p1; pos >= p2; pos -= 1) {
      servo1.write(pos);
      servo2.write(pos);
      servo3.write(pos); // Move third servo
      servo4.write(pos); // Move fourth servo
      delay(5); // Fast movement
    }
  } else if (p2 > p1) {
    for (int pos = p1; pos <= p2; pos += 1) {
      servo1.write(pos);
      servo2.write(pos);
      servo3.write(pos); // Move third servo
      servo4.write(pos); // Move fourth servo
      delay(5); // Fast movement
    }
  }
  delay(500); // Short pause after movement
}

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  Serial.println("ESP32 Quad Servo Custom Rotation Control"); // Updated title for 4 servos

  // Allow allocation of all timers for servos
  // This is important for ESP32 to manage multiple servos
  // ESP32 has 16 PWM channels, so 4 servos are well within its capabilities.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Attach the servo objects to their respective pins
  // The min and max pulse widths (500-2500) are standard for most servos.
  // You might need to adjust these values slightly for your specific servos
  // to get the full range of motion without straining them.
  servo1.setPeriodHertz(50); // Standard 50Hz servo frequency
  servo1.attach(servoPin1, 500, 2500); // Attach servo1 to servoPin1
  Serial.print("Servo 1 attached to pin: ");
  Serial.println(servoPin1);

  servo2.setPeriodHertz(50); // Standard 50Hz servo frequency
  servo2.attach(servoPin2, 500, 2500); // Attach servo2 to servoPin2
  Serial.print("Servo 2 attached to pin: ");
  Serial.println(servoPin2);

  servo3.setPeriodHertz(50); // Standard 50Hz servo frequency
  servo3.attach(servoPin3, 500, 2500); // Attach servo3 to servoPin3
  Serial.print("Servo 3 attached to pin: ");
  Serial.println(servoPin3);

  servo4.setPeriodHertz(50); // Standard 50Hz servo frequency
  servo4.attach(servoPin4, 500, 2500); // Attach servo4 to servoPin4
  Serial.print("Servo 4 attached to pin: ");
  Serial.println(servoPin4);

  // Set initial position for all servos to 90 degrees
  servo1.write(90);
  servo2.write(90);
  servo3.write(90); // Initialize third servo
  servo4.write(90); // Initialize fourth servo
  Serial.println("All Servos initialized to 90 degrees.");
  delay(1000); // Give servos time to reach the initial position
}

void loop() {
  // --- Movement 1: From 90 to 135 degrees ---
  Serial.println("Moving servos from 90 to 135 degrees...");
  motor_control(90, 135);

  // --- Movement 2: From 135 back to 90 degrees ---
  Serial.println("Moving servos from 135 back to 90 degrees...");
  motor_control(135, 90);

  // --- Movement 3: From 90 to 55 degrees ---
  Serial.println("Moving servos from 90 to 55 degrees...");
  motor_control(90, 55);

  // --- Movement 4: From 55 back to 90 degrees ---
  Serial.println("Moving servos from 55 back to 90 degrees...");
  motor_control(55, 90);
}
