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
#define AZI_ENC_A       PB9    // Encoder Channel A Green
#define AZI_ENC_B       PB8    // Encoder Channel B White
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
#define ALT_ENC_A       PB10    // Encoder Channel A Green
#define ALT_ENC_B       PB11   // Encoder Channel B White
#define ALT_LIM_BTN     PA1   // Calibration switch
#define ALT_ENC_MAX     400


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

#define AZI_KP 2
#define AZI_KI 0.2
#define AZI_KD 0.2


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

#define ALT_KP 2
#define ALT_KI 0.2
#define ALT_KD 0.2
#define TARGET_MASK 0b0000_0011_1111_1111
#define MOTOR_MASK  0b0000_0100_0000_0000



/***********************
 *                     *
 *     STATE FLOW      * 
 *                     *
 **********************/
int state = 0;

#define AZI_CALIBRATION 0
#define AZI_MOVE_45     1
#define ALT_CALIBRATION 2
#define OPERATE         4



/***********************
 *                     *
 *     PID CONTROL     * 
 *                     *
 **********************/
#define MAX_SPEED 100  // Maximum speed (PWM value)
#define MIN_SPEED 0    // Minimum speed
#define TOLERANCE 1    // Tolerance in encoder units

bool azi_reached_goal = false;
bool alt_reached_goal = false;

double azi_error = 0;
double azi_previous_error = 0;
double azi_integral = 0;
double azi_derivative = 0;
double azi_output = 0;
unsigned long azi_current_time, azi_previous_time;
double azi_elapsed_time;
int azi_target = 300; // Set this to the 45 degree angle

double alt_error = 0;
double alt_previous_error = 0;
double alt_integral = 0;
double alt_derivative = 0;
double alt_output = 0;
unsigned long alt_current_time, alt_previous_time;
double alt_elapsed_time;

int alt_target = 20;




inline
void updateAziEncoder()
{
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

  Serial.println(azi_last_enc);
}

inline
void updateAltEncoder()
{
  int msb = digitalRead(ALT_ENC_A); //MSB = most significant bit
  int lsb = digitalRead(ALT_ENC_B); //LSB = least significant bit

  int encoded = (msb << 1) |lsb; //converting the 2 pin value to single number
  int sum  = (alt_last_enc << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) 
  {
    alt_enc_val --;
  }
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
  {
    alt_enc_val++;
  }

  alt_last_enc = encoded; //store this value for next time

  Serial.println(alt_enc_val);
}

inline
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

inline
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

inline
void aziCalibration() {

  while (digitalRead(AZI_LIM_BTN) == LOW) 
  {
    moveMotor(AZIMUTH, false, 5);  // Move in CW azi_direction
  }
  stopMotor(AZIMUTH);
  azi_enc_val = 0;  // Reset encoder position
}
inline 
bool altCalibrationCmd()
{
  if (digitalRead(ALT_LIM_BTN) == LOW) 
  {
    moveMotor(ALTITUDE, false, 5);  // Move in CW azi_direction
    return false;
  }
  else 
  {
    stopMotor(ALTITUDE);
    alt_enc_val = 0;  // Reset encoder position
    return true;
  }
}

inline
bool aziPidCmd(int angle) {

  azi_previous_time = millis();

  azi_error = angle - azi_enc_val;
  
  // IMPORTANT - instert serial communication here

  // If error is within tolerance, exit loop
  if (abs(azi_error) <= TOLERANCE) {
    return true;
  }

  azi_current_time = millis();
  azi_elapsed_time = (double)(azi_current_time - azi_previous_time) / 1000.0; // Convert ms to seconds

  // Calculate integral
  azi_integral += azi_error * azi_elapsed_time;

  // Calculate derivative
  azi_derivative = (azi_error - azi_previous_error) / azi_elapsed_time;

  // Compute PID output
  azi_output = AZI_KP * azi_error + AZI_KI * azi_integral + AZI_KD * azi_derivative;

  // Constrain output to max and min speed
  azi_output = constrain(azi_output, -MAX_SPEED, MAX_SPEED);

  // Determine motor direction and speed
  bool direction = azi_output >= 0; // True for CW, False for CCW
  int speed = abs((int)azi_output);
  //Serial.println(speed);

  // Ensure speed is within bounds
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);

  // Move motor
  moveMotor(AZIMUTH ,direction, speed);

  // Update variables for next loop
  azi_previous_error = azi_error;
  azi_previous_time = azi_current_time;
  
  return false;
  //delay(5);
}
inline
bool altPidCommand(int angle) {

  alt_previous_time = millis();

  alt_error = angle - alt_enc_val;
  
  // IMPORTANT - instert serial communication here

  // If error is within tolerance, exit loop
  if (abs(alt_error) <= TOLERANCE) {
    return true;
  }

  alt_current_time = millis();
  alt_elapsed_time = (double)(alt_current_time - alt_previous_time) / 1000.0; // Convert ms to seconds

  // Calculate integral
  alt_integral += alt_error * alt_elapsed_time;

  // Calculate derivative
  alt_derivative = (alt_error - alt_previous_error) / alt_elapsed_time;

  // Compute PID output
  alt_output = ALT_KP * alt_error + ALT_KI * alt_integral + ALT_KD * alt_derivative;

  // Constrain output to max and min speed
  alt_output = constrain(alt_output, -MAX_SPEED, MAX_SPEED);

  // Determine motor direction and speed
  bool direction = alt_output >= 0; // True for CW, False for CCW
  int speed = abs((int)alt_output);
  //Serial.println(speed);

  // Ensure speed is within bounds
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);

  // Move motor
  moveMotor(ALTITUDE ,direction, speed);

  // Update variables for next loop
  alt_previous_error = alt_error;
  alt_previous_time = alt_current_time;

  return false;
  //delay(5);
}


void setup() {

  /* Azimuth Control Setup */
  pinMode(AZI_MOTOR, OUTPUT);
  pinMode(AZI_MOTOR_DIR, OUTPUT);
  pinMode(AZI_ENC_A, INPUT); 
  pinMode(AZI_ENC_B, INPUT);
  //attachInterrupt(digitalPinToInterrupt(AZI_ENC_A), updateAziEncoder, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(AZI_ENC_B), updateAziEncoder, CHANGE);
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


void loop() 
{
  uint8_t upper_byte, lower_byte;
  bool is_azi;
  /*
   * CONTROL SQUENCE
   *
   * calibration:
   * 1) Move azimuth to 0 position
   * 2) Move azimuth to 45 degree position and hold
   * 3) Move altitude to 0 position
   * 4) Move altitude to 10 degree position
   * 
   * Operation:
   */
  switch (state)
  {
  case AZI_CALIBRATION:
    aziCalibration();
    state++;
    break;
  case AZI_MOVE_45:
    azi_reached_goal = aziPidCmd(azi_target); // NOTE need to make function that converts ticks to degrees
    if (azi_reached_goal)
    {
      state++;
      azi_reached_goal = false;
    }
    break;
  case ALT_CALIBRATION:
    aziPidCmd(300);     // Keep azimuth steady;

    alt_reached_goal = altCalibrationCmd();
    if (alt_reached_goal)
    {
      state++;
      alt_reached_goal = false;
    }
    break;
  case OPERATE:
      aziPidCmd(azi_target);
      altPidCommand(alt_target);

      if (Serial.available() >= 2)
      {
        upper_byte = Serial.read();
        lower_byte = Serial.read();

        is_azi = upper_byte >> 2;

        if (is_azi)
        {
          azi_target = (upper_byte << 8) | (lower_byte);
        }
        else
        {
          alt_target = (upper_byte << 8) | (lower_byte);
        }
        

      }
    break;  
  default:
    break;
  }
}
