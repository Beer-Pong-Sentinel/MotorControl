#include <Arduino.h>

#define AZI_MOTOR       PA8    // Note that the pin must be PWM compatible
#define AZI_MOTOR_PWM   PA_8    // has to be in this weird underscore format
#define AZI_MOTOR_DIR   PB13 

bool azi_motor_dir = HIGH;


void setup() {
  // put your setup code here, to run once:
  pinMode(AZI_MOTOR, OUTPUT);
  pinMode(AZI_MOTOR_DIR, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  pwm_start(AZI_MOTOR_PWM, 20000, 20,
        	TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);
}
