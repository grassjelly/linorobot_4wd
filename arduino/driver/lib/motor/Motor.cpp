#include "Arduino.h"
#include "Motor.h"
#include "lino_base_config.h"

Motor::Motor(int pwm_pin, int motor_pinA, int motor_pinB)
{
    pinMode(pwm_pin, OUTPUT);
    pinMode(motor_pinA, OUTPUT);
    pinMode(motor_pinB, OUTPUT);
    
    _pwm_pin = pwm_pin;
    _motor_pinA = motor_pinA;
    _motor_pinB = motor_pinB;
}

void Motor::calculate_rpm(long current_encoder_ticks, unsigned long dt)
{
    //this function calculates the motor's RPM based on encoder ticks and delta time

    //convert the time from milliseconds to minutes
    double dtm = (double)dt / 60000;
    //calculate change in number of ticks from the encoder
    double delta_ticks = current_encoder_ticks - _previous_encoder_ticks;
    //calculate wheel's speed (RPM)
    current_rpm = (delta_ticks / ticks_per_rev) / dtm;
    _previous_encoder_ticks = current_encoder_ticks;   
}

void Motor::calculate_pwm()
{
    //this function implements a PID controller and calculates the PWM required to drive the motor
    double pid;
    double new_rpm;
    double error;
    
    //calculate the error ()
    error = required_rpm - current_rpm;
    //calculate the overall error
    _total_pid_error += error;
    //PID controller
    pid = Kp * error  + Ki * _total_pid_error + Kd * (error - _previous_pid_error);
    _previous_pid_error = error;
    //adds the calculated PID value to the required rpm for error compensation
    new_rpm = constrain(double(pwm) * max_rpm / 255 + pid, -max_rpm, max_rpm);
    //maps rpm to PWM signal, where 255 is the max possible value from an 8 bit controller
    pwm = (new_rpm / max_rpm) * 255;
}

void Motor::spin()
{
    //this function spins the motor based on calculated PWM
    if(pwm >= 0)
    {
        digitalWrite(_motor_pinA , HIGH);
        digitalWrite(_motor_pinB , LOW);    
    }
    else
    {
        digitalWrite(_motor_pinA , LOW);
        digitalWrite(_motor_pinB , HIGH);
    }
    analogWrite(_pwm_pin , abs(pwm));
}

