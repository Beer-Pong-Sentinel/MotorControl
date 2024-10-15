#include <Arduino.h>

// Pin definitions
#define AZI_MOTOR       PA8    // PWM compatible pin for motor control
#define AZI_MOTOR_PWM   PA_8   // Motor PWM control
#define AZI_MOTOR_DIR   PB13   // Motor direction control

#define ENC_AZI_A       PB9    // Encoder Channel A
#define ENC_AZI_B       PB8    // Encoder Channel B

#define LIMIT_AZI_BTN   PB12   // Calibration switch
#define STATUS_LED      PC13   // Status LED

// Variables
volatile long encoderPosition = 0;  // Current encoder position
const int encoderResolution = 600;  // Pulses per revolution
bool direction = true;  // Motor direction: true = CW, false = CCW
int targetAngle = 0;  // Target angle to move to after calibration

int buttonState = 0;

// Function to read encoder
void readEncoder() {
  static bool lastA = LOW;
  bool A = digitalRead(ENC_AZI_A);
  bool B = digitalRead(ENC_AZI_B);
  
  // Check for rising edge on channel A
  if (A != lastA && A == HIGH) {
    if (B == LOW) {
      encoderPosition++;
      direction = true;  // CW
    } else {
      encoderPosition--;
      direction = false; // CCW
    }
  }
  lastA = A;
}


void moveMotor(bool dir, int pwmValue) {
  digitalWrite(AZI_MOTOR_DIR, dir);  // Set direction
 ///analogWrite(AZI_MOTOR_PWM, pwmValue);  // Set motor speed
  pwm_start(AZI_MOTOR_PWM, 500, 50,
        	TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
}

void stopMotor() {
  pwm_start(AZI_MOTOR_PWM, 500, 0,
        	TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
}

void moveToCalibrationSwitch() {
  // Rotate motor until calibration switch is hit
  while (digitalRead(LIMIT_AZI_BTN) == LOW) {
    moveMotor(true, 50);  // Move in CW direction
  }
  stopMotor();
  encoderPosition = 0;  // Reset encoder position
  Serial.println("Calibration complete, encoder reset.");
}

void moveToAngle(int angle) {
  // Calculate target position based on angle
  long targetPosition = (long)encoderResolution * angle / 360;

  // Move motor until target position is reached
  while (encoderPosition != targetPosition) {
    if (encoderPosition < targetPosition) {
      moveMotor(true, 150);  // Move CW
    } else {
      moveMotor(false, 150);  // Move CCW
    }
  }
  stopMotor();
  Serial.print("Reached angle: ");
  Serial.println(angle);
}

void setup() {
  // Motor control pins
  pinMode(AZI_MOTOR, OUTPUT);
  pinMode(AZI_MOTOR_DIR, OUTPUT);
  
  // Encoder pins
  //pinMode(ENC_AZI_A, INPUT);
  //pinMode(ENC_AZI_B, INPUT);
  
  // Calibration switch
  pinMode(LIMIT_AZI_BTN, INPUT_PULLDOWN);
  
  // Status LED
  pinMode(STATUS_LED, OUTPUT);
  
  // Attach interrupt for encoder readings
  //attachInterrupt(digitalPinToInterrupt(ENC_AZI_A), readEncoder, CHANGE);
  
  // Serial for debugging
  Serial.begin(9600);
}

void loop() {
  // Example flow:
  
  // Step 1: Move to calibration switch
  moveToCalibrationSwitch();
  
  
  // Step 2: Set target angle (can be provided via Serial or predefined)
  //targetAngle = 90;  // Example angle: 90 degrees
  //buttonState = digitalRead(LIMIT_AZI_BTN);
  

  // Button is working
  /*if (buttonState == HIGH) {
    digitalWrite(STATUS_LED, HIGH);

  } else {
    digitalWrite(STATUS_LED, LOW);
  }*/

  // Step 3: Move to target angle
  //moveToAngle(targetAngle);

  // Turn on status LED to indicate task completion
 // digitalWrite(STATUS_LED, HIGH);
  
  // Prevent further action in loop
  //while (true);
  while(true);
}
