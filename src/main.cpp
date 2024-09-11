#include <Arduino.h>

#define AZI_MOTOR       PA8    // Note that the pin must be PWM compatible
#define AZI_MOTOR_PWM   PA_8    // has to be in this weird underscore format
#define AZI_MOTOR_DIR   PB13 

#define ENC_A           PB9
#define ENC_B           PB8

bool azi_motor_dir = HIGH;

volatile unsigned int temp, counter = 0; //This variable will increase or decrease depending on the rotation of encoder

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(ENC_B)==LOW) {
    counter++;
  }else{
    counter--;
  }
  }
   
  void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(ENC_A)==LOW) {
    counter--;
  }else{
    counter++;
  }
  }

void setup() {
  Serial.begin (9600);
  // put your setup code here, to run once:
  pinMode(AZI_MOTOR, OUTPUT);
  pinMode(AZI_MOTOR_DIR, OUTPUT);

  pinMode(ENC_A, INPUT_PULLUP); // internal pullup input pin 2 
  
  pinMode(ENC_B, INPUT_PULLUP); // internalเป็น pullup input pin 3
  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(digitalPinToInterrupt(ENC_A), ai0, RISING);
   
  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(digitalPinToInterrupt(ENC_B), ai1, RISING);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  pwm_start(AZI_MOTOR_PWM, 20000, 20,
        	TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);

  if( counter != temp ){
    Serial.println (counter);
    temp = counter;
  }
}

