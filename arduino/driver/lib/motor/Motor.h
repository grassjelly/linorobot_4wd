#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor
{
    public:
        double current_rpm;
        double required_rpm;
        int pwm;     
        
        static float Kp;
        static float Kd;
        static float Ki;
        
        Motor(int pwm_pin, int motor_pinA, int motor_pinB);
        void calculate_rpm(long current_encoder_ticks, unsigned long dt);
        void calculate_pwm();
        void spin();        
        
    private:
        int _pwm_pin;
        int _motor_pinA;
        int _motor_pinB;
        
        double _previous_pid_error; 
        double _total_pid_error; 
        long _previous_encoder_ticks; 
};

#endif
