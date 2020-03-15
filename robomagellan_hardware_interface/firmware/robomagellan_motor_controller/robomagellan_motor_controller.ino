#include "Vex393EncoderMotor.hpp"

const int left_servo_pin = 3;
const int right_servo_pin = 5;
const int battery_pin = A0;
const double battery_factor = 7.2 / 1024;
const double gear_factor = -1.0 / 3.0;

Vex393EncoderMotor left_motor(gear_factor, 2.0, 0.0, 0.2);
Vex393EncoderMotor right_motor(gear_factor, 2.0, 0.0, 0.2);

void setup() {
  Wire.begin();

  left_motor.setup(left_servo_pin);
  right_motor.setup(right_servo_pin);

  pinMode(battery_pin, INPUT);
  
  Serial.begin(115200);
}

void loop() {
  if(Serial.available())
  {
    parseCommand();
    sendReply();
  }
  
  left_motor.update();
  right_motor.update();

  delay(10);
}

void parseCommand()
{
  while(Serial.read() != '$');
  left_motor.setSpeed(Serial.parseFloat());
  right_motor.setSpeed(-1*Serial.parseFloat());
  Serial.read(); // Take the newline out of the receive buffer
}

void sendReply()
{
  Serial.print("$");
  Serial.print(left_motor.getPosition());
  Serial.print(",");
  Serial.print(-1*right_motor.getPosition());
  Serial.print(",");
  Serial.print(left_motor.getSpeed());
  Serial.print(",");
  Serial.print(-1*right_motor.getSpeed());
  Serial.print(",");
  Serial.print(battery_factor * analogRead(battery_pin));
  Serial.println();
}
