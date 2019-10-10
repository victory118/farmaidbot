/**
 * @file drv8833_motor.h
 * @brief Motor device driver for the IBT-2 dual motor driver
 * @author Victor Yu
 */

#ifndef IBT2_MOTOR_H
#define IBT2_MOTOR_H

// Control logic:
// Forward: L_PWM: PWM pin (0-100% duty cycle), R_PWM: PWM pin (0% duty cycle)
// Reverse: L_PWM: PWM pin (0% duty cycle), R_PWM: PWM pin (0-100% duty cycle)
// VCC: 5V (Arduino), GND: GND (Arduino)
// L_EN: 5V (Arduino), R_EN: 5V (Arduino)

namespace Farmaid
{
    
    class Motor
    {
    public:
        /*
         * @brief Class constructor
         * @param dir_pin_ the direction pin 
         * @param pwm_pin the PWM pin
         */
         Motor(byte pwm_f_pin, byte pwm_r_pin, unsigned int max_command, float no_load_rps)
            : pwm_f_pin_(pwm_f_pin), pwm_r_pin_(pwm_r_pin),
              max_command_(max_command), no_load_rps_(no_load_rps),
              command_(0)
         {
             pinMode(pwm_f_pin_, OUTPUT);
             pinMode(pwm_r_pin_, OUTPUT);
         }

        void set_command(float value)
        {

            // input value is limited to range [-1.0, 1.0] and then mapped to range [-max_command_, +max_command_]
            float factor = max(min(value, 1.0f), -1.0f);

            if (factor >= 0) // forward direction
            {
                command_ = (unsigned int)(max_command_ * factor);
                analogWrite(pwm_r_pin_, 0);
                analogWrite(pwm_f_pin_, command_);
            } else { // reverse direction
                command_ = (unsigned int)(max_command_ * abs(factor));
                analogWrite(pwm_f_pin_, 0);
                analogWrite(pwm_r_pin_, command_);
            }
        }

        unsigned int command_;
        const float no_load_rps_; // max speed with no load [rad/s]
         
    private:
        const int pwm_f_pin_; // forward direction PWM pin
        const int pwm_r_pin_; // reverse direction PWM pin
        const int max_command_; //  0-255 for 0-100% PWM duty cycle
    };
};

#endif
