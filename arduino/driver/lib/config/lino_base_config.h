#ifndef LINO_BASE_CONFIG_H
#define LINO_BASE_CONFIG_H

#define DEBUG 0

#define k_p 0.4 // P constant
#define k_i 0.0 // I constant
#define k_d 1.0 // D constant

#define pi 3.1415926 
#define two_pi 6.2831853

//define your motors' specs here
#define encoder_pulse 55 //encoder's number of ticks per revolution 
#define gear_ratio 30 //motor's gear ratio
#define max_rpm 330 //motor's maximum RPM

//define your robot base's specs here
#define wheel_diameter 0.135 //wheel's diameter in meters
#define wheel_width 0.053 //wheel's width in meters
#define track_width 0.31 // width of the plate you are using

//don't change this if you followed the schematic diagram
//ENCODER PINS
#define FRONT_LEFT_ENCODER_A 3
#define FRONT_LEFT_ENCODER_B 2 
// Right side encoders pins
#define FRONT_RIGHT_ENCODER_A 6
#define FRONT_RIGHT_ENCODER_B 7

#define REAR_LEFT_ENCODER_A 5
#define REAR_LEFT_ENCODER_B 4
// Right side encoders pins
#define REAR_RIGHT_ENCODER_A 8
#define REAR_RIGHT_ENCODER_B 9

//don't change this if you followed the schematic diagram
//MOTOR PINS
//Left Motor
#define front_left_motor_pwm 23
#define front_left_motor_in_1 17
#define front_left_motor_in_2 16

#define rear_left_motor_pwm 22
#define rear_left_motor_in_1 15
#define rear_left_motor_in_2 14

#define front_right_motor_pwm 21
#define front_right_motor_in_1 13
#define front_right_motor_in_2 12

#define rear_right_motor_pwm 20
#define rear_right_motor_in_1 11
#define rear_right_motor_in_2 10
#endif