#include <Arduino.h>

// Pin definitions
#define AZI_MOTOR       PA8    // PWM compatible pin for motor control
#define AZI_MOTOR_PWM   PA_8   // Motor PWM control
#define AZI_MOTOR_DIR   PB13   // Motor direction control

#define ENC_AZI_A       PB9    // Encoder Channel A
#define ENC_AZI_B       PB8    // Encoder Channel B

#define LIMIT_AZI_BTN   PB12   // Calibration switch
#define STATUS_LED      PC13   // Status LED

#define ENCODER_MAX   855

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

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue ++;

  lastEncoded = encoded; //store this value for next time
}

// Function to read encoder

void moveMotor(bool dir, int pwmValue) {
  digitalWrite(AZI_MOTOR_DIR, dir);  // Set direction
 ///analogWrite(AZI_MOTOR_PWM, pwmValue);  // Set motor speed
  pwm_start(AZI_MOTOR_PWM, 500, pwmValue,
        	TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
}

void stopMotor() {
  pwm_start(AZI_MOTOR_PWM, 500, 0,
        	TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
}

void moveToCalibrationSwitch() {
  // Rotate motor until calibration switch is hit
  while (digitalRead(LIMIT_AZI_BTN) == LOW) {
    moveMotor(false, 5);  // Move in CW direction
  }
  stopMotor();
  encoderValue = 0;  // Reset encoder position
  Serial.println("Calibration complete, encoder reset.");
  Serial.println(encoderValue);

}


void moveToAngle(int angle, double Kp, double Ki, double Kd) {

  double error = 0;
  double previous_error = 0;
  double integral = 0;
  double derivative = 0;
  double output = 0;

  unsigned long currentTime, previousTime;
  double elapsedTime;

  int maxSpeed = 50; // Maximum speed (PWM value)
  int minSpeed = 0;   // Minimum speed
  int tolerance = 1;  // Tolerance in encoder units

  previousTime = millis();

  // Move motor until target position is reached
  while (true) {
    // Calculate error
    error = angle - encoderValue;

    // If error is within tolerance, exit loop
    if (abs(error) <= tolerance) {
      break;
    }

    currentTime = millis();
    elapsedTime = (double)(currentTime - previousTime) / 1000.0; // Convert ms to seconds

    // Calculate integral
    integral += error * elapsedTime;

    // Calculate derivative
    derivative = (error - previous_error) / elapsedTime;

    // Compute PID output
    output = Kp * error + Ki * integral + Kd * derivative;

    // Constrain output to max and min speed
    output = constrain(output, 0, maxSpeed);

    // Determine motor direction and speed
    bool direction = output >= 0; // True for CW, False for CCW
    int speed = abs((int)output);

    // Ensure speed is within bounds
    speed = constrain(speed, minSpeed, maxSpeed);

    // Move motor
    moveMotor(direction, speed);

    // Update variables for next loop
    previous_error = error;
    previousTime = currentTime;

    // Debugging output
    Serial.print("Encoder Value: ");
    Serial.print(encoderValue);
    Serial.print(" | Error: ");
    Serial.print(error);
    Serial.print(" | Output: ");
    Serial.println(output);

  }

  stopMotor();
  Serial.print("Reached angle: ");
  Serial.println(angle);
}


void setup() {
  pinMode(AZI_MOTOR, OUTPUT);
  pinMode(AZI_MOTOR_DIR, OUTPUT);
  

  pinMode(ENC_AZI_A, INPUT); 
  pinMode(ENC_AZI_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_AZI_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_AZI_B), updateEncoder, CHANGE);
  
  pinMode(LIMIT_AZI_BTN, INPUT_PULLDOWN);
  
  pinMode(STATUS_LED, OUTPUT);
  

  Serial.begin(9600);
}


unsigned long startTime;
unsigned long endTime;
unsigned long shortestTime = 999999; // Initialize with a large value
float bestKp = 0, bestKi = 0, bestKd = 0;

// PID ranges and increments
float kp_min = 0.5, kp_max = 10.0, kp_step = 0.5;
float ki_min = 0.1, ki_max = 5.0, ki_step = 0.1;
float kd_min = 0.1, kd_max = 5.0, kd_step = 0.1;

void loop() {
  // First, move to calibration switch
  moveToCalibrationSwitch();
  
  // // Loop over PID values to find the quickest
  // for (float Kp = kp_min; Kp <= kp_max; Kp += kp_step) {
  //   for (float Ki = ki_min; Ki <= ki_max; Ki += ki_step) {
  //     for (float Kd = kd_min; Kd <= kd_max; Kd += kd_step) {
        
  //       // Record the start time
  //       startTime = millis();
        
  //       // Call the moveToAngle function with current Kp, Ki, Kd
  //       moveToAngle(200, Kp, Ki, Kd);
        
  //       // Record the end time
  //       endTime = millis();
        
  //       // Calculate time taken
  //       unsigned long timeTaken = endTime - startTime;
        
  //       // Print current PID parameters and time taken
  //       Serial.print("Kp: ");
  //       Serial.print(Kp);
  //       Serial.print(", Ki: ");
  //       Serial.print(Ki);
  //       Serial.print(", Kd: ");
  //       Serial.print(Kd);
  //       Serial.print(" -> Time: ");
  //       Serial.println(timeTaken);
        
  //       // Check if this is the shortest time
  //       if (timeTaken < shortestTime) {
  //         shortestTime = timeTaken;
  //         bestKp = Kp;
  //         bestKi = Ki;
  //         bestKd = Kd;
  //       }
  //     }
  //   }
  // }
  moveToAngle(200, 1, 0.5, 0.5);
  
  // After all iterations, print the best parameters
  Serial.println("Best parameters:");
  Serial.print("Kp: ");
  Serial.print(bestKp);
  Serial.print(", Ki: ");
  Serial.print(bestKi);
  Serial.print(", Kd: ");
  Serial.print(bestKd);
  Serial.print(" -> Shortest time: ");
  Serial.println(shortestTime);
  while(true);
}
