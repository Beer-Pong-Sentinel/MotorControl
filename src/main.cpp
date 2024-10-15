#include <Arduino.h>

#define AZI_MOTOR       PA8    // Note that the pin must be PWM compatible
#define AZI_MOTOR_PWM   PA_8    // has to be in this weird underscore format
#define AZI_MOTOR_DIR   PB13 

#define ENC_AZI_A           PB9
#define ENC_AZI_B           PB8

#define LIMIT_AZI_BTN       PB12

#define STATUS_LED          PC13

bool azi_motor_dir = HIGH;
int buttonState = 0;


volatile unsigned int temp, counter = 0; //This variable will increase or decrease depending on the rotation of encoder

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(ENC_AZI_B)==LOW) {
    counter++;
  }else{
    counter--;
  }
  }
   
  void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(ENC_AZI_A)==LOW) {
    counter--;
  }else{
    counter++;
  }
  }

void setup() {
  Serial.begin (9600);

  pinMode(LIMIT_AZI_BTN, INPUT_PULLDOWN);
  pinMode(STATUS_LED, OUTPUT);


  // put your setup code here, to run once:
  pinMode(AZI_MOTOR, OUTPUT);
  pinMode(AZI_MOTOR_DIR, OUTPUT);

  pinMode(ENC_AZI_A, INPUT_PULLUP); // internal pullup input pin 2 
  
  pinMode(ENC_AZI_B, INPUT_PULLUP); // internalเป็น pullup input pin 3
  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(digitalPinToInterrupt(ENC_AZI_A), ai0, RISING);
   
  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(digitalPinToInterrupt(ENC_AZI_B), ai1, RISING);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  pwm_start(AZI_MOTOR_PWM, 20000, 20,
        	TimerCompareFormat_t::PERCENT_COMPARE_FORMAT);

  if( counter != temp ){
    Serial.println (counter);
    temp = counter;
  }

    // Read the state of the button
  buttonState = digitalRead(LIMIT_AZI_BTN);

  // If the button is pressed (LOW, because of pull-up resistor)
  if (buttonState == HIGH) {
    // Turn on the LED
    digitalWrite(STATUS_LED, HIGH);
  } else {
    // Turn off the LED
    digitalWrite(STATUS_LED, LOW);
  }

  // Small delay for debouncing (can be adjusted if needed)
  //delay(50);
}

