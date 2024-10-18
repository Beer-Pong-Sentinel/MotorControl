#include <Arduino.h>

// Pin definitions
#define AZI_MOTOR       PA8    // PWM compatible pin for motor control
#define AZI_MOTOR_PWM   PA_8   // Motor PWM control
#define AZI_MOTOR_DIR   PB13   // Motor direction control

#define ENC_AZI_A       PB9    // Encoder Channel A
#define ENC_AZI_B       PB8    // Encoder Channel B

#define LIMIT_AZI_BTN   PB12   // Calibration switch
#define STATUS_LED      PC13   // Status LED

bool direction = true;  // Motor direction: true = CW, false = CCW
int targetAngle = 0;  // Target angle to move to after calibration

int buttonState = 0;


volatile int lastEncoded = 0;
volatile long encoderValue = 0;

long lastencoderValue = 0;

int lastMSB = 0;
int lastLSB = 0;

void updateEncoder()
{
  int MSB = digitalRead(ENC_AZI_A); //MSB = most significant bit
  int LSB = digitalRead(ENC_AZI_B); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;

  lastEncoded = encoded; //store this value for next time
}

// Function to read encoder

void moveMotor(bool dir, int pwmValue) {
  digitalWrite(AZI_MOTOR_DIR, dir);  // Set direction
 ///analogWrite(AZI_MOTOR_PWM, pwmValue);  // Set motor speed
  pwm_start(AZI_MOTOR_PWM, 500, 5,
        	TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
}

void stopMotor() {
  pwm_start(AZI_MOTOR_PWM, 500, 0,
        	TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
}

void moveToCalibrationSwitch() {
  // Rotate motor until calibration switch is hit
  while (digitalRead(LIMIT_AZI_BTN) == LOW) {
    moveMotor(false, 50);  // Move in CW direction
  }
  stopMotor();
  encoderValue = 0;  // Reset encoder position
  Serial.println("Calibration complete, encoder reset.");
}

void moveToAngle(int angle) {
  // Calculate target position based on angle
  long targetPosition = (long)encoderValue * angle / 360;

  // Move motor until target position is reached
  while (encoderValue < 200) {
    if (encoderValue < 200) {
      moveMotor(false, 150);  // Move CW
    } else {
      moveMotor(true, 150);  // Move CCW
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
  

  pinMode(ENC_AZI_A, INPUT); 
  pinMode(ENC_AZI_B, INPUT);

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(digitalPinToInterrupt(ENC_AZI_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_AZI_B), updateEncoder, CHANGE);
  
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
  moveToAngle(200);

  // Turn on status LED to indicate task completion
 // digitalWrite(STATUS_LED, HIGH);
  
  // Prevent further action in loop
  //while (true);
  while(true);
}
