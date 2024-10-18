#include <Arduino.h>

// Define the encoder pins
#define ENCODER_PIN_A PB11
#define ENCODER_PIN_B PB10

volatile int lastEncoded = 0;
volatile long encoderValue = 0;

long lastencoderValue = 0;

int lastMSB = 0;
int lastLSB = 0;

void updateEncoder()
{
  int MSB = digitalRead(ENCODER_PIN_A); //MSB = most significant bit
  int LSB = digitalRead(ENCODER_PIN_B); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;

  lastEncoded = encoded; //store this value for next time
}

void setup()
{
  Serial.begin (9600);

  pinMode(ENCODER_PIN_A, INPUT); 
  pinMode(ENCODER_PIN_B, INPUT);

  //digitalWrite(ENCODER_PIN_A, HIGH); //turn pullup resistor on
  //digitalWrite(ENCODER_PIN_B, HIGH); //turn pullup resistor on

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), updateEncoder, CHANGE);

}

void loop()
{ 

  Serial.println(encoderValue);
  delay(1000); //just here to slow down the output, and show it will work  even during a delay
}


