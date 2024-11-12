#include <Arduino.h>

#define AZIMUTH   true
#define ALTITUDE  false

/**********************
 *                    *
 *     AZIMUTH PINS   *
 *                    *
 **********************/
#define AZI_MOTOR       PA8    // PWM compatible pin for motor control
#define AZI_MOTOR_PWM   PA_8   // Motor PWM control
#define AZI_MOTOR_DIR   PB13   // Motor azi_direction control
#define AZI_ENC_A       PB9    // Encoder Channel A
#define AZI_ENC_B       PB8    // Encoder Channel B
#define AZI_LIM_BTN     PB12    // Calibration switch
#define AZI_ENC_MAX     855

/**********************
 *                    *
 *   ALTITUDE PINS    *
 *                    *
 **********************/
#define ALT_MOTOR       PA8    // PWM compatible pin for motor control
#define ALT_MOTOR_PWM   PA_8   // Motor PWM control
#define ALT_MOTOR_DIR   PB13   // Motor azi_direction control
#define ALT_ENC_A       PB9    // Encoder Channel A
#define ALT_ENC_B       PB8    // Encoder Channel B
#define ALT_LIM_BTN     PB12    // Calibration switch
#define ALT_ENC_MAX     855


#define STATUS_LED      PC13   // Status LED


/***********************
 *                     *
 *  AZIMUTH VARIABLES  *
 *                     *
 **********************/
bool          azi_direction     = true;   // Motor azi_direction: true = CW, false = CCW
int           azi_target_angle  = 0;      // Target angle to move to after calibration
int           azi_cal_btn       = 0;
volatile int  azi_last_enc      = 0;
volatile long azi_enc_val       = 0;
double        azi_kp            = 2;
double        azi_ki            = 0.2;
double        azi_kd            = 0.2;

/***********************
 *                     *
 * ALTITUDE VARIABLES  *
 *                     *
 **********************/
bool          alt_direction     = true;   // Motor azi_direction: true = CW, false = CCW
int           alt_target_angle  = 0;      // Target angle to move to after calibration
int           alt_cal_btn       = 0;
volatile int  alt_last_enc      = 0;
volatile long alt_enc_val       = 0;
double        alt_kp            = 2;
double        alt_ki            = 0.2;
double        alt_kd            = 0.2;

inline
void updateAziEncoder()
{
  int last_enc = azi_last_enc;
  int msb = digitalRead(AZI_ENC_A); //MSB = most significant bit
  int lsb = digitalRead(AZI_ENC_B); //LSB = least significant bit

  int encoded = (msb << 1) |lsb; //converting the 2 pin value to single number
  int sum  = (azi_last_enc << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) 
  {
    azi_enc_val --;
  }
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
  {
    azi_enc_val++;
  }

  azi_last_enc = encoded; //store this value for next time
}

inline
void updateAltEncoder()
{
  int last_enc = alt_last_enc;
  int msb = digitalRead(ALT_ENC_A); //MSB = most significant bit
  int lsb = digitalRead(ALT_ENC_B); //LSB = least significant bit

  int encoded = (msb << 1) |lsb; //converting the 2 pin value to single number
  int sum  = (azi_last_enc << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) 
  {
    alt_enc_val --;
  }
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
  {
    alt_enc_val++;
  }

  alt_last_enc = encoded; //store this value for next time
}

void moveMotor(bool is_azi, bool dir, int pwmValue) 
{
  if (is_azi)
  {
    digitalWrite(AZI_MOTOR_DIR, dir);

    pwm_start(  AZI_MOTOR_PWM, 
                500, 
                pwmValue,
        	      TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
  }
  else 
  {
    digitalWrite(ALT_MOTOR_DIR, dir);

    pwm_start(  ALT_MOTOR_PWM, 
                500, 
                pwmValue,
        	      TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
  }
}


void stopMotor(bool is_azi) {
  if (is_azi)
  {
      pwm_start(  AZI_MOTOR_PWM, 
                  500, 
                  0,
        	        TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
  }
  else
  {
      pwm_start(  ALT_MOTOR_PWM, 
                  500, 
                  0,
        	        TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
  }

}

void moveToCalibrationSwitch(bool is_azi) {
  if (is_azi)
  {
    while (digitalRead(AZI_LIM_BTN) == LOW) 
    {
      moveMotor(AZIMUTH, false, 5);  // Move in CW azi_direction
    }
    stopMotor(AZIMUTH);
    azi_enc_val = 0;  // Reset encoder position
  }
  else
  {
    while (digitalRead(ALT_LIM_BTN) == LOW) 
    {
      moveMotor(ALTITUDE, false, 5);  // Move in CW azi_direction
    }
    stopMotor(ALTITUDE);
    alt_enc_val = 0;  // Reset encoder position
  }
}


void moveToAngle(bool is_azi, int angle, double Kp, double Ki, double Kd) {

  double error = 0;
  double previous_error = 0;
  double integral = 0;
  double derivative = 0;
  double output = 0;

  unsigned long currentTime, previousTime;
  double elapsedTime;

  int maxSpeed = 100; // Maximum speed (PWM value)
  int minSpeed = 0;   // Minimum speed
  int tolerance = 1;  // Tolerance in encoder units

  previousTime = millis();

  // Move motor until target position is reached
  while (true) {
    // Calculate error
    is_azi ? error = angle - azi_enc_val : error = angle - alt_enc_val;
    
    // IMPORTANT - instert serial communication here

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
    output = constrain(output, -maxSpeed, maxSpeed);

    // Determine motor direction and speed
    bool direction = output >= 0; // True for CW, False for CCW
    int speed = abs((int)output);
    //Serial.println(speed);

    // Ensure speed is within bounds
    speed = constrain(speed, minSpeed, maxSpeed);

    // Move motor
    moveMotor(is_azi ,direction, speed);

    // Update variables for next loop
    previous_error = error;
    previousTime = currentTime;
    
    delay(5);

  }

  stopMotor(is_azi);

}


void setup() {

  /* Azimuth Control Setup */
  pinMode(AZI_MOTOR, OUTPUT);
  pinMode(AZI_MOTOR_DIR, OUTPUT);
  pinMode(AZI_ENC_A, INPUT); 
  pinMode(AZI_ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(AZI_ENC_A), updateAziEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(AZI_ENC_B), updateAziEncoder, CHANGE);
  pinMode(AZI_LIM_BTN, INPUT_PULLDOWN);

  /* Altitude Control Setup */
  pinMode(ALT_MOTOR, OUTPUT);
  pinMode(ALT_MOTOR_DIR, OUTPUT);
  pinMode(ALT_ENC_A, INPUT); 
  pinMode(ALT_ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ALT_ENC_A), updateAltEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ALT_ENC_B), updateAltEncoder, CHANGE);
  pinMode(ALT_LIM_BTN, INPUT_PULLDOWN);


  pinMode(STATUS_LED, OUTPUT);
  

  Serial.begin(9600);
}


void loop() {
  
  while(true);
}
