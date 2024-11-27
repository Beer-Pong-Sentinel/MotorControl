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
#define AZI_ENC_MAX     850

/**********************
 *                    *
 *   ALTITUDE PINS    *
 *                    *
 **********************/
#define ALT_MOTOR       PA3    // PWM compatible pin for motor control
#define ALT_MOTOR_PWM   PA_3   // Motor PWM control
#define ALT_MOTOR_DIR   PA4   // Motor azi_direction control
#define ALT_ENC_A       PB10    // Encoder Channel A Green
#define ALT_ENC_B       PB11   // Encoder Channel B White
#define ALT_LIM_BTN     PA1     // Calibration switch
#define ALT_ENC_MAX     630


#define STATUS_LED      PC13   // Status LED


/***********************
 *                     *
 *  AZIMUTH VARIABLES  *
 *                     *
 **********************/
int           azi_cal_btn       = 0;
volatile int  azi_last_enc      = 0;
volatile long azi_enc_val       = 0;
bool azi_direction     = 0; // True for CW, False for CCW
int azi_speed          = 0;

#define AZI_KP        0.1
#define AZI_KI        0.0
#define AZI_KD        0.01
#define AZI_INIT_POS  420


/***********************
 *                     *
 * ALTITUDE VARIABLES  *
 *                     *
 **********************/
int           alt_cal_btn       = 0;
volatile int  alt_last_enc      = 0;
volatile long alt_enc_val       = 0;
bool alt_direction     = 0; // True for CW, False for CCW
int alt_speed          = 0;

#define ALT_KP        1.5
#define ALT_KI        1
#define ALT_KD        2.1
#define ALT_INIT_POS  300
#define ALT_SEC_POS   100



/***********************
 *                     *
 *     STATE FLOW      * 
 *                     *
 **********************/
#define AZI_CALIBRATION 0
#define AZI_MOVE_45     1
#define ALT_CALIBRATION 2
#define ALT_INIT        3
#define ALT_SEC         4
#define OPERATE         5

int state = AZI_CALIBRATION;


/***********************
 *                     *
 *     PID CONTROL     * 
 *                     *
 **********************/
#define MAX_SPEED 15  // Maximum speed (PWM value)
#define MIN_SPEED 0    // Minimum speed
#define TOLERANCE 2    // Tolerance in encoder units
#define ALT_TOLERANCE 2

bool azi_reached_goal = false;
bool alt_reached_goal = false;

double azi_error = 0;
double azi_previous_error = 0;
double azi_integral = 0;
double azi_derivative = 0;
double azi_output = 0;
unsigned long azi_current_time = 0, azi_previous_time = millis();
double azi_elapsed_time = 1e-6;
int azi_target = AZI_INIT_POS; // Set this to the 45 degree angle

double alt_error = 0;
double alt_previous_error = 0;
double alt_integral = 0;
double alt_derivative = 0;
double alt_output = 0;
unsigned long alt_current_time = 0, alt_previous_time = millis();
double alt_elapsed_time = 1e-6;
int alt_target = ALT_INIT_POS;


unsigned long previousMillis = 0; // Stores the last time the action was performed
const unsigned long interval = 1000; // Interval (1 second)

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

  //Serial.println(azi_last_enc);
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

  //Serial.println(alt_enc_val);
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

    delay(1);

  }
  else 
  {
    digitalWrite(ALT_MOTOR_DIR, dir);

    pwm_start(  ALT_MOTOR_PWM, 
                500, 
                pwmValue,
        	      TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
    //delayMicroseconds(100);
    delay(1);

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
      delayMicroseconds(50);

  }
  else
  {
      pwm_start(  ALT_MOTOR_PWM, 
                  500, 
                  0,
        	        TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
      delayMicroseconds(50);

  }

}

inline
void aziHoming() {

  while (digitalRead(AZI_LIM_BTN) == LOW) 
  {
    moveMotor(AZIMUTH, false, 7);  // Move in CW azi_direction
  }
  stopMotor(AZIMUTH);
  azi_enc_val = 0;  // Reset encoder position
}

inline 
bool altHomingCmd()
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
  azi_direction = azi_output >= 0; // True for CW, False for CCW
  azi_speed = abs((int)azi_output);

  // Update variables for next command
  azi_previous_error = azi_error;
  azi_previous_time = azi_current_time;

  moveMotor(AZIMUTH ,azi_direction, azi_speed);

  return false;
}

bool altPidCommand(int angle) {

  alt_error = angle - alt_enc_val;
  
  // If error is within tolerance, exit loop
  if (abs(alt_error) <= ALT_TOLERANCE) {
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
  alt_direction = alt_output >= 0; // True for CW, False for CCW
  alt_speed = abs((int)alt_output);

  // Update variables for next loop
  alt_previous_error = alt_error;
  alt_previous_time = alt_current_time;

  moveMotor(ALTITUDE ,alt_direction, alt_speed);

  delayMicroseconds(50);

  return false;
}

// void receiveData()
// {
//   uint8_t azi_upper_byte, azi_lower_byte;
//   uint8_t alt_upper_byte, alt_lower_byte;
//       if (Serial.available() >= 4)
//       {
//         azi_upper_byte = Serial.read();
//         azi_lower_byte = Serial.read();
//         alt_upper_byte = Serial.read();
//         alt_lower_byte = Serial.read();

//         azi_target = (azi_upper_byte << 8) | (azi_lower_byte);
//         alt_target = (alt_upper_byte << 8) | (alt_lower_byte);
//         Serial.write(azi_upper_byte); 
//         delayMicroseconds(10);
//         Serial.write(azi_lower_byte);
//         delayMicroseconds(10);
//         Serial.write(alt_upper_byte); 
//         delayMicroseconds(10);
//         Serial.write(alt_lower_byte);

//       }
// }

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


void loop() 
{

  bool is_azi;
  unsigned long currentMillis = millis(); // Get the current time

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
    aziHoming(); // Blocking function
    state = AZI_MOVE_45;
    break;
  case AZI_MOVE_45:
    azi_reached_goal = aziPidCmd(azi_target); 

    if (azi_reached_goal)
    {
      state = ALT_CALIBRATION;
      azi_reached_goal = false;
      stopMotor(AZIMUTH);
    }
    break;
  case ALT_CALIBRATION:
    aziPidCmd(azi_target);     // Keep azimuth steady while altitude is homing 

    alt_reached_goal = altHomingCmd();
    if (alt_reached_goal)
    {
      state=ALT_INIT;
      alt_reached_goal = false;
    }
    break;
  case ALT_INIT:
      alt_reached_goal = altPidCommand(ALT_INIT_POS);
      aziPidCmd(azi_target);

      if (alt_reached_goal)
      {
        state=ALT_SEC;
        alt_reached_goal = false;
      }

    break;  
    case ALT_SEC:
      alt_reached_goal = altPidCommand(ALT_SEC_POS);
      aziPidCmd(azi_target);

      if (alt_reached_goal)
      {
        stopMotor(AZIMUTH);
        stopMotor(ALTITUDE);
        state=OPERATE;
        alt_reached_goal = false;
      }
    break;
    case OPERATE:
      alt_reached_goal = altPidCommand(alt_target);
      azi_reached_goal = aziPidCmd(azi_target);
      if (alt_reached_goal == true && azi_reached_goal == true)
      {
        stopMotor(AZIMUTH);
        stopMotor(ALTITUDE);
      }
    break;  
  default:
    break;
  }

  /*
   * One second intervals to send out information
   */
  // if (currentMillis - previousMillis >= interval) 
  // {
  //   previousMillis = currentMillis; // Update the last action time
    
  //   // Action to perform every second
  //   Serial.println(alt_enc_val);
  //   Serial.println(alt_speed);
  //   Serial.println(alt_direction);
  //   Serial.println(alt_error);
  //   Serial.println(alt_target);

  // }
}
