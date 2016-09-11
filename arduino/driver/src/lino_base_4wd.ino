/*
 Copyright (c) 2016, Juan Jimeno
 Credits to:
 Sungjik https://github.com/sungjik 
 Tony Baltowski https://github.com/tonybaltovski
 
 All rights reserved. 
 
 Redistribution and use in source and binary forms, with or without 
 modification, are permitted provided that the following conditions are met: 
 
 * Redistributions of source code must retain the above copyright notice, 
 this list of conditions and the following disclaimer. 
 * Redistributions in binary form must reproduce the above copyright 
 notice, this list of conditions and the following disclaimer in the 
 documentation and/or other materials provided with the distribution. 
 * Neither the name of  nor the names of its contributors may be used to 
 endorse or promote products derived from this software without specific 
 prior written permission. 
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 POSSIBILITY OF SUCH DAMAGE. 
 
 */
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <ros.h>

//header file for publishing "rpm"
#include <geometry_msgs/Vector3Stamped.h>

//header file for cmd_subscribing to "cmd_vel"
#include <geometry_msgs/Twist.h>

//header file for pid server
#include <lino_pid/linoPID.h>

//header files for imu
#include <ros_arduino_msgs/RawImu.h>
#include <geometry_msgs/Vector3.h>

#include <ros/time.h>

#include <Wire.h>

#include "imu_configuration.h"
#include "lino_base_config.h"

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include "Motor.h"

#define IMU_PUBLISH_RATE 10 //hz
#define VEL_PUBLISH_RATE 10 //hz
#define COMMAND_RATE 10 //hz
#define DEBUG_RATE 5


//add your motor objects here
Motor front_left_motor(front_left_motor_pwm, front_left_motor_in_1, front_left_motor_in_2);
Motor rear_left_motor(rear_left_motor_pwm, rear_left_motor_in_1, rear_left_motor_in_2);
Motor front_right_motor(front_right_motor_pwm, front_right_motor_in_1, front_right_motor_in_2);
Motor rear_right_motor(rear_right_motor_pwm, rear_right_motor_in_1, rear_right_motor_in_2);

//add your encoder objects here
Encoder front_left_encoder(FRONT_LEFT_ENCODER_A,FRONT_LEFT_ENCODER_B);
Encoder front_right_encoder(FRONT_RIGHT_ENCODER_A,FRONT_RIGHT_ENCODER_B);
Encoder rear_left_encoder(REAR_LEFT_ENCODER_A,REAR_LEFT_ENCODER_B);
Encoder rear_right_encoder(REAR_RIGHT_ENCODER_A,REAR_RIGHT_ENCODER_B);

//function prototypes
void check_imu();
void publish_imu();
void publish_linear_velocity(unsigned long);
void get_speed(unsigned long dt);
void get_pwm();
void move_base();


//callback function prototypes
void command_callback( const geometry_msgs::Twist& cmd_msg);
void pid_callback( const lino_pid::linoPID& pid);

unsigned long lastMilli = 0;       
unsigned long lastMilliPub = 0;
unsigned long previous_command_time = 0;
unsigned long previous_control_time = 0;
unsigned long publish_vel_time = 0;
unsigned long previous_imu_time = 0;
unsigned long previous_debug_time = 0;

bool is_first = true;

float Motor::Kp = k_p;
float Motor::Kd = k_d;
float Motor::Ki = k_i;

char buffer[50];

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", command_callback);
ros::Subscriber<lino_pid::linoPID> pid_sub("pid", pid_callback);

ros_arduino_msgs::RawImu raw_imu_msg;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);

geometry_msgs::Vector3Stamped raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

void setup()
{
//   initialize_motors();
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(pid_sub);
  nh.subscribe(cmd_sub);
  nh.advertise(raw_vel_pub);
  nh.advertise(raw_imu_pub);

  while (!nh.connected())
  {
    nh.spinOnce();
  }
  nh.loginfo("Connected to microcontroller...");
  nh.loginfo("ROS Arduino IMU started.");
  
#if defined(WIRE_T3)
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);
#else
  Wire.begin();
#endif
  delay(5);
}

void loop()
{
  //this block publishes velocity based on defined rate
  if ((millis() - publish_vel_time) >= (1000 / VEL_PUBLISH_RATE))
  {
    unsigned long current_time = millis();
    publish_linear_velocity(current_time - publish_vel_time);
    publish_vel_time = millis();
  }

  //this block drives the robot based on defined rate
  if ((millis() - previous_control_time) >= (1000 / COMMAND_RATE))
  {
    unsigned long current_time = millis();
    unsigned long dt = current_time - previous_control_time;
    get_speed(dt);
    get_pwm();
    move_base();
    previous_control_time = millis();
  }

  //this block stops the motor when no command is received
  if ((millis() - previous_command_time) >= 400)
  {
    front_left_motor.required_rpm = 0;
    rear_left_motor.required_rpm = 0;
    front_right_motor.required_rpm = 0;    
    rear_right_motor.required_rpm = 0;
    front_left_motor.pwm = 0;
    rear_left_motor.pwm = 0;
    front_right_motor.pwm = 0;    
    rear_right_motor.pwm = 0;
  }

  //this block publishes the IMU data based on defined rate
  if ((millis() - previous_imu_time) >= (1000 / IMU_PUBLISH_RATE))
  {
    //sanity check if the IMU exits
    if (is_first)
    {
      check_imu();
    }
    else
    {
      //publish the IMU data
      publish_imu();
    }
    previous_imu_time = millis();
  }
  
  //this block displays the encoder readings. change DEBUG to 0 if you don't want to display
  if(DEBUG)
  {
    if ((millis() - previous_debug_time) >= (1000 / DEBUG_RATE))
    {
      sprintf (buffer, "Encoder FrontLeft: %ld", front_left_encoder.read());
      nh.loginfo(buffer);
      sprintf (buffer, "Encoder RearLeft: %ld", rear_left_encoder.read());
      nh.loginfo(buffer);
      sprintf (buffer, "Encoder FrontRight: %ld", front_right_encoder.read());
      nh.loginfo(buffer);
      sprintf (buffer, "Encoder RearRight: %ld", rear_right_encoder.read());
      nh.loginfo(buffer);
      previous_debug_time = millis();
    }
  }
  //call all the callbacks waiting to be called
  nh.spinOnce();
}

void pid_callback( const lino_pid::linoPID& pid) 
{
  //callback function every time PID constants are received from lino_pid for tuning
  //this callback receives pid object where P,I, and D constants are stored
    Motor::Kp = pid.p;
    Motor::Kd = pid.d;
    Motor::Ki = pid.i;
}

void command_callback( const geometry_msgs::Twist& cmd_msg)
{
  //callback function every time linear and angular speed is received from 'cmd_vel' topic
  //this callback function receives cmd_msg object where linear and angular speed are stored

  previous_command_time = millis();
  double linear_vel = cmd_msg.linear.x;
  double angular_vel = cmd_msg.angular.z;
  //convert m/s to m/min
  double linear_vel_mins = linear_vel * 60;
  //convert rad/s to rad/min
  double angular_vel_mins = angular_vel * 60;
  //calculate the wheel's circumference
  double circumference = pi * wheel_diameter;
  //calculate the tangential velocity of the wheel if the robot's rotating where Vt = ω * radius
  //this is to compensate to slippage of wheels
  float wheel_compensate = 2;
  double tangential_vel = (angular_vel_mins * (track_width / 2)) * wheel_compensate;

  //calculate and assign desired RPM for each motor
  //left side
  front_left_motor.required_rpm = (linear_vel_mins / circumference) - (tangential_vel / circumference);
  rear_left_motor.required_rpm = front_left_motor.required_rpm;
  
  //right side
  front_right_motor.required_rpm = (linear_vel_mins / circumference) + (tangential_vel / circumference);
  rear_right_motor.required_rpm = front_right_motor.required_rpm;
}

void get_speed(unsigned long dt)
{
    //calculate motor's current speed
    front_left_motor.calculate_rpm(front_left_encoder.read(), dt);
    rear_left_motor.calculate_rpm(rear_left_encoder.read(), dt);
    front_right_motor.calculate_rpm(front_right_encoder.read(), dt);
    rear_right_motor.calculate_rpm(rear_right_encoder.read(), dt);    
}

void get_pwm()
{
    //calculate how much PWM is needed based on required RPM and error over time
    front_left_motor.calculate_pwm();
    rear_left_motor.calculate_pwm();    
    front_right_motor.calculate_pwm();
    rear_right_motor.calculate_pwm();     
}

void move_base()
{
    //move the wheels based on calculated pwm
    front_left_motor.spin();
    rear_left_motor.spin();    
    front_right_motor.spin();
    rear_right_motor.spin();     
}

void publish_linear_velocity(unsigned long time)
{
  // this function publishes the linear speed of the robot
  
  //calculate the average RPM 
  double average_rpm = (front_left_motor.current_rpm + rear_left_motor.current_rpm + front_right_motor.current_rpm + rear_right_motor.current_rpm) / 4; // RPM
  //convert revolutions per minute to revolutions per second
  double average_rps = average_rpm / 60; // RPS
  //calculate linear speed
  double linear_velocity = (average_rps * (wheel_diameter * pi)); // m/s 
  
  //fill in the object 
  raw_vel_msg.header.stamp = nh.now();
  raw_vel_msg.vector.x = linear_velocity;
  raw_vel_msg.vector.y = 0.00;
  raw_vel_msg.vector.z = double(time) / 1000;
  //publish raw_vel_msg object to ROS
  raw_vel_pub.publish(&raw_vel_msg);
  nh.spinOnce();
}

void check_imu()
{
  //this function checks if IMU is present
  raw_imu_msg.accelerometer = check_accelerometer();
  raw_imu_msg.gyroscope = check_gyroscope();
  raw_imu_msg.magnetometer = check_magnetometer();

  if (!raw_imu_msg.accelerometer)
  {
    nh.logerror("Accelerometer NOT FOUND!");
  }

  if (!raw_imu_msg.gyroscope)
  {
    nh.logerror("Gyroscope NOT FOUND!");
  }

  if (!raw_imu_msg.magnetometer)
  {
    nh.logerror("Magnetometer NOT FOUND!");
  }
  
  is_first = false;
}

void publish_imu()
{
  //this function publishes raw IMU reading
  raw_imu_msg.header.stamp = nh.now();
  raw_imu_msg.header.frame_id = "imu_link";
  //measure accelerometer
  if (raw_imu_msg.accelerometer)
  {
    measure_acceleration();
    raw_imu_msg.raw_linear_acceleration = raw_acceleration;
  }

  //measure gyroscope
  if (raw_imu_msg.gyroscope)
  {
    measure_gyroscope();
    raw_imu_msg.raw_angular_velocity = raw_rotation;
  }
  
  //measure magnetometer
  if (raw_imu_msg.magnetometer)
  {
    measure_magnetometer();
    raw_imu_msg.raw_magnetic_field = raw_magnetic_field;
  }
  //publish raw_imu_msg object to ROS
  raw_imu_pub.publish(&raw_imu_msg);
}

