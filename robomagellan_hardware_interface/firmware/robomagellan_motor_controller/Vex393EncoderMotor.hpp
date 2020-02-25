#include <Wire.h>
#include <I2CEncoder.h>
#include <Servo.h>
#include <PID_v1.h>


class Vex393EncoderMotor
{
public:
  Vex393EncoderMotor(double gearFactor = 1.0, double Kp = 0.5, double Ki = 0.0, double Kd = 0.0)
    : pid(&current_speed, &pid_out, &set_speed, Kp, Ki, Kd, REVERSE),
      gear_factor(gearFactor)
  {
  }

  void setup(int servoPin)
  {
    servo.attach(servoPin);
    encoder.init(MOTOR_393_TORQUE_ROTATIONS, MOTOR_393_TIME_DELTA);
    pid.SetOutputLimits(-90.0, 90.0);
    pid.SetMode(MANUAL);
  }

  void update()
  {
    updateSpeed();
    updatePwm();
  }

  void setSpeed(double rpm)
  {
    set_speed = rpm;
    if(set_speed < 0.01 && set_speed > -0.01)
    {
      pid.SetMode(MANUAL);
    }
    else
    {
      pid.SetMode(AUTOMATIC);
    }
  }

  double getPosition()
  {
    return previous_position;
  }

  double getSpeed()
  {
    return current_speed;
  }
  
public:
  Servo servo;
  I2CEncoder encoder;
  PID pid;

  void updateSpeed()
  {
    unsigned long current_time = micros();
    double current_position = encoder.getPosition();
    double delta_pos = current_position - previous_position;
    if(delta_pos < 1.0 && delta_pos > -1.0)
    {
      double delta_t_secs = (current_time - previous_speed_time) / 1000000.0;
      current_speed = (delta_pos / delta_t_secs) * gear_factor;
    }
    previous_position = current_position;
    previous_speed_time = current_time;
  }
  
  void updatePwm()
  {
    if(set_speed < 0.01 && set_speed > -0.01)
    {
      pwm = STOPPED_PWM;
    }
    else
    {
      pid.Compute();
      pwm += pid_out;
      pwm = constrain(pwm, MIN_PWM, MAX_PWM);
    }
    
    servo.write((int)pwm);
    //servo.write(110);
  }

  double set_speed{0.0};
  double current_speed{0.0};
  double pid_out{0.0};
  double pwm{0.0};
  double gear_factor{1.0};
  
  double previous_position = 0.0;
  unsigned long previous_speed_time = 0;

  static const double STOPPED_PWM{90.0};
  static const double MAX_PWM{150.0};
  static const double MIN_PWM{30.0};
};
