#ifndef MEGA_4WD_H
#define MEGA_4WD_H

#include "ls7184_encoder.h"
#include "ibt2_motor.h"
#include "pid_controller.h"
#include "robot4wd.h"

const float ctrl_freq = 100.0; // [Hz] frequency of motor control loop and encoder sampling
const float ctrl_period = 1.0 / ctrl_freq;
const float ctrl_period_micros = ctrl_period * 1e6;

const float print_freq = 10.0; // [Hz] frequency of printing debug messages
const float print_period = 1.0 / print_freq;
const float print_period_micros = print_period * 1e6;

const float ros_freq = 30.0; // [Hz] freqency of serial communication with ROS
const float ros_period = 1.0 / ros_freq;
const float ros_period_micros = ros_period * 1e6;

const float wheelbase = 0.255; // [m]; 0.255 m (farmbot v1); 0.3747 m (farmbot v2)
const float wheel_radius = 0.0635; // [m]
const float no_load_rps = 8.4; // determined experimentally in [m/s] --> [rad/s]

const float Tf_enc = 5.0*ctrl_period; // encoder velocity low-pass filter time constant

// Encoder counter variables are global because they use hardware interrupts
volatile int fleft_encoder_count = 0;
volatile bool fleft_encoder_change_flag = false;
volatile int fright_encoder_count = 0;
volatile bool fright_encoder_change_flag = false;
volatile int rleft_encoder_count = 0;
volatile bool rleft_encoder_change_flag = false;
volatile int rright_encoder_count = 0;
volatile bool rright_encoder_change_flag = false;

// Initialize encoders: Encoder(byte clk_pin, byte dir_pin, float counts_per_rev, float Ts, float Tf, volatile int *count_ptr)
Farmaid::Encoder fleft_encoder(19, 23, 748.65*2, ctrl_period, Tf_enc, &fleft_encoder_count);
Farmaid::Encoder fright_encoder(18, 22, 748.65*2, ctrl_period, Tf_enc, &fright_encoder_count);
Farmaid::Encoder rleft_encoder(3, 24, 748.65*2, ctrl_period, Tf_enc, &rleft_encoder_count);
Farmaid::Encoder rright_encoder(2, 25, 748.65*2, ctrl_period, Tf_enc, &rright_encoder_count);

// Initialize motors: Motor(byte pwm_f_pin, byte pwm_r_pin, unsigned int max_command, float no_load_rps)
Farmaid::Motor fleft_motor(9, 8, 255, no_load_rps);
Farmaid::Motor fright_motor(7, 6, 255, no_load_rps);
Farmaid::Motor rleft_motor(11, 10, 255, no_load_rps);
Farmaid::Motor rright_motor(5, 4, 255, no_load_rps);

// Initialize PID controllers: PidController(float Kp, float Ki, float Kd, float Tf, float Ts)
Farmaid::PidController fleft_pid(2.0, 0.0, 0.0, 0.0, ctrl_period);
Farmaid::PidController fright_pid(2.0, 0.0, 0.0, 0.0, ctrl_period);
Farmaid::PidController rleft_pid(2.0, 0.0, 0.0, 0.0, ctrl_period);
Farmaid::PidController rright_pid(2.0, 0.0, 0.0, 0.0, ctrl_period);

// Initialize robot
Farmaid::Robot4WD robot(fleft_encoder, fright_encoder,
                        rleft_encoder, rright_encoder,
                        fleft_motor, fright_motor,
                        rleft_motor, rright_motor,
                        fleft_pid, fright_pid,
                        rleft_pid, rright_pid,
                        wheelbase, wheel_radius);

void ReadEncoders()
{
    // This reads directly from the global variables
    if (fleft_encoder_change_flag) {
        fleft_encoder_change_flag = false;
        Serial.print("Front Left encoder count = ");
        Serial.println(fleft_encoder_count);
    }
    
    if (fright_encoder_change_flag) {
        fright_encoder_change_flag = false;
        Serial.print("Front Right encoder count = ");
        Serial.println(fright_encoder_count);
    }
    
    if (rleft_encoder_change_flag) {
        rleft_encoder_change_flag = false;
        Serial.print("Rear Left encoder count = ");
        Serial.println(rleft_encoder_count);
    }
    
    if (rright_encoder_change_flag) {
        rright_encoder_change_flag = false;
        Serial.print("Rear Right encoder count = ");
        Serial.println(rright_encoder_count);
    }
}

void FLeftEncoderInterrupt() {
  if (digitalRead(fleft_encoder.dir_pin_) == HIGH) {
    fleft_encoder_count++;
  }
  else {
    fleft_encoder_count--;
  }
  fleft_encoder_change_flag = true;
}

void FRightEncoderInterrupt() {
  if (digitalRead(fright_encoder.dir_pin_) == LOW) {
    fright_encoder_count++;
  }
  else {
    fright_encoder_count--;
  }
  fright_encoder_change_flag = true;
}

void RLeftEncoderInterrupt() {
  if (digitalRead(rleft_encoder.dir_pin_) == HIGH) {
    rleft_encoder_count++;
  }
  else {
    rleft_encoder_count--;
  }
  rleft_encoder_change_flag = true;
}

void RRightEncoderInterrupt() {
  if (digitalRead(rright_encoder.dir_pin_) == LOW) {
    rright_encoder_count++;
  }
  else {
    rright_encoder_count--;
  }
  rright_encoder_change_flag = true;
}

#endif
