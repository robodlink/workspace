// Include the ESP32Servo library
// You might need to install this library through the Arduino IDE's Library Manager.
// Search for "ESP32Servo" and install it.
#include <ESP32Servo.h>
#include <vector> // Required for std::vector
#include <array>  // Required for std::array

// Define the pins connected to the servo signal wires
const int servoPin1 = 12; // GPIO pin for the first servo (e.g., Left Top Hip)
const int servoPin2 = 13; // GPIO pin for the second servo (e.g., Left Bottom Foot)
const int servoPin3 = 14; // GPIO pin for the third servo (e.g., Right Top Hip)
const int servoPin4 = 27; // GPIO pin for the fourth servo (e.g., Right Bottom Foot)

// Create Servo objects for each servo
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

// Array to hold the current position of each servo
// This is crucial for smooth individual movements from their last known position
int currentServoPositions[4] = {90, 90, 90, 90}; // Initialize to 90 degrees

// Define the movement pattern using std::vector of std::array
// Each std::array<int, 4> represents {servo1_angle_change, servo2_angle_change, servo3_angle_change, servo4_angle_change}
// The base angle for each servo is 90 degrees. The value in the array is added to 90.
std::vector<std::array<int, 4>> moves = {
    // {L_Top, L_Bot, R_Top, R_Bot} Angle Change from 90 deg base
   
    {0,  25,0 , 0},      // Example: Left Bottom moves +20 deg from 90 (to 110)
    {35,  25, 35, 0},    // Example: Both Tops move +35 deg from 90 (to 125), Left Bottom stays at 110, Right Bottom stays at 90
    {35  ,-10 ,  35, 0},      // Example: Left Bottom moves +20 deg from its current 110 (to 130), others stay
    {35, -10, 35 , -30},     // Example: Right Bottom moves -20 deg from 90 (to 70), others stay
    {-35, -10, -35,-30},  // Example: Both Tops move -35 deg from their current (125 to 90), Left Bot moves +1 (130 to 131), Right Bot moves -20 (70 to 50)
    {-35 , -10 ,-35, 0},
    {-35,-10,-35,0}    // Example: Right Bottom moves -20 deg from its current 50 (to 30), others stay
};

// Function to move multiple servos smoothly to target angles
// It moves each servo from its current position to its new target position.
// This allows for different servos to move different amounts simultaneously.
void moveServosToAngles(int targetAngle1, int targetAngle2, int targetAngle3, int targetAngle4, int stepDelayMs = 10) {
    // Determine the maximum number of steps required across all servos for smooth movement
    int maxSteps = 0;
    maxSteps = max(maxSteps, abs(targetAngle1 - currentServoPositions[0]));
    maxSteps = max(maxSteps, abs(targetAngle2 - currentServoPositions[1]));
    maxSteps = max(maxSteps, abs(targetAngle3 - currentServoPositions[2]));
    maxSteps = max(maxSteps, abs(targetAngle4 - currentServoPositions[3]));

    if (maxSteps == 0) return; // No movement needed

    // Calculate step increments for each servo
    float step1 = (float)(targetAngle1 - currentServoPositions[0]) / maxSteps;
    float step2 = (float)(targetAngle2 - currentServoPositions[1]) / maxSteps;
    float step3 = (float)(targetAngle3 - currentServoPositions[2]) / maxSteps;
    float step4 = (float)(targetAngle4 - currentServoPositions[3]) / maxSteps;

    // Iterate through steps, moving each servo
    for (int i = 1; i <= maxSteps; i++) {
        servo1.write(round(currentServoPositions[0] + step1 * i));
        servo2.write(round(currentServoPositions[1] + step2 * i));
        servo3.write(round(currentServoPositions[2] + step3 * i));
        servo4.write(round(currentServoPositions[3] + step4 * i));
        delay(stepDelayMs);
    }

    // After movement, update current positions to the final target angles
    currentServoPositions[0] = targetAngle1;
    currentServoPositions[1] = targetAngle2;
    currentServoPositions[2] = targetAngle3;
    currentServoPositions[3] = targetAngle4;
    
    delay(1000); // Small pause after reaching target (adjust as needed)
}


void setup() {
    // Initialize serial communication for debugging
    Serial.begin(115200);
    Serial.println("ESP32 4-DOF Bipedal Robot Control"); // Updated title for 4 servos

    // Allow allocation of all timers for servos
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    // Attach the servo objects to their respective pins
    // Default min/max pulse widths (500-2500) for 0-180 degrees
    servo1.setPeriodHertz(50); // Standard 50Hz servo frequency
    servo1.attach(servoPin1, 500, 2500);
    Serial.print("Servo 1 (L_Top) attached to pin: "); Serial.println(servoPin1);

    servo2.setPeriodHertz(50);
    servo2.attach(servoPin2, 500, 2500);
    Serial.print("Servo 2 (L_Bot) attached to pin: "); Serial.println(servoPin2);

    servo3.setPeriodHertz(50);
    servo3.attach(servoPin3, 500, 2500);
    Serial.print("Servo 3 (R_Top) attached to pin: "); Serial.println(servoPin3);

    servo4.setPeriodHertz(50);
    servo4.attach(servoPin4, 500, 2500);
    Serial.print("Servo 4 (R_Bot) attached to pin: "); Serial.println(servoPin4);

    // Set initial position for all servos to 90 degrees (neutral/base)
    servo1.write(currentServoPositions[0]);
    servo2.write(currentServoPositions[1]);
    servo3.write(currentServoPositions[2]);
    servo4.write(currentServoPositions[3]);
    Serial.println("All Servos initialized to 90 degrees.");
    delay(1500); // Give servos time to reach the initial position
}

void loop() {
    Serial.println("\n--- Starting Movement Pattern ---");

    // Loop through each movement definition in the 'moves' vector
    for (const auto& move_data : moves) { // Using const auto& for efficient iteration
        int angle_change1 = move_data[0];
        int angle_change2 = move_data[1];
        int angle_change3 = move_data[2];
        int angle_change4 = move_data[3];

        // Calculate target angles based on the 90-degree base and angle change
        // IMPORTANT: If you want these angle changes to be *relative* to the *current* position,
        // you would do:
        // int target1 = currentServoPositions[0] + angle_change1;
        // int target2 = currentServoPositions[1] + angle_change2;
        // ... and so on.
        // As per your previous comment "Angle Change from 90 deg base", the current approach is correct.
        int target1 = 90 + angle_change1; 
        int target2 = 90 + angle_change2;
        int target3 = 90 + angle_change3;
        int target4 = 90 + angle_change4;

        Serial.print("Moving to: S1="); Serial.print(target1);
        Serial.print(", S2="); Serial.print(target2);
        Serial.print(", S3="); Serial.print(target3);
        Serial.print(", S4="); Serial.print(target4);
        Serial.println("");

        // Move all servos to their respective target angles
        moveServosToAngles(target1, target2, target3, target4);
    }
    
    Serial.println("--- Movement Pattern Complete. Returning to 90. ---");
    // After the sequence, return all servos to the 90-degree base position
    moveServosToAngles(90, 90, 90, 90, 20); // Slower return to base
    delay(1000); // Pause before repeating the loop
}
