#ifndef TEST_H
#define TEST_H

extern unsigned long curr_micros;
extern unsigned long prev_print_micros;
extern const float print_period_micros;

namespace Farmaid
{
void TestEncoderClass(Encoder &left_encoder, Encoder &right_encoder)
{

    if (curr_micros - prev_print_micros >= print_period_micros)
    {   
            
        left_encoder.ProcessMeasurement();
        right_encoder.ProcessMeasurement();

        if (left_encoder.count_ != left_encoder.prev_count_)
        {
            Serial.print("Left encoder count (prev) = ");
            Serial.println(left_encoder.prev_count_);
            Serial.print("Left encoder count = ");
            Serial.println(left_encoder.count_);
        }

        if (right_encoder.count_ != right_encoder.prev_count_)
        {
            Serial.print("Right encoder count (prev) = ");
            Serial.println(right_encoder.prev_count_);
            Serial.print("Right encoder count = ");
            Serial.println(right_encoder.count_);
        }

        prev_print_micros = curr_micros;
    }        
}

void TestMotorOpenLoop(Motor &motor, float factor)
{

    float control = 0;

    if (curr_micros < 2e6) { control = 0 * factor; }
    else if (curr_micros < 4e6) { control = 0.5 * factor; }
    else if (curr_micros < 6e6) { control = 1.0 * factor; }
    else if (curr_micros < 8e6) { control = 0.5 * factor; }
    else { control = 0.0 * factor; }

    motor.set_command(control);

    if (curr_micros - prev_print_micros >= print_period_micros)
    { 
        Serial.println(control);
        prev_print_micros = curr_micros;
    }
}

void TestMotorPositionControl(Motor &motor, Encoder &encoder, PidController &pid, float factor)
{

    float setpoint = 0;

    if (curr_micros < 4e6) { setpoint = 0 * factor; }
    else if (curr_micros < 8e6) { setpoint = 0.5 * factor; }
    else if (curr_micros < 12e6) { setpoint = 1.0 * factor; }
    else if (curr_micros < 16e6) { setpoint = 0.5 * factor; }
    else { setpoint = 0.0 * factor; }

    // Process encoder measurement
    encoder.ProcessMeasurement();

    // Get the current motor position in revolutions
    float curr_pos = encoder.count_ / encoder.counts_per_rev_;

    // Compute controller command based on desired and current position
    float control = pid.ComputeControl(setpoint, curr_pos, 0);

    // Send controller command to motor
    motor.set_command(control);

    if (curr_micros - prev_print_micros >= print_period_micros)
    {
        Serial.print(2.0);
        Serial.print(" ");
        Serial.print(-2.0);
        Serial.print(" ");
        Serial.print(setpoint);
        Serial.print(" ");
        Serial.println(curr_pos);
        prev_print_micros = curr_micros;
    }
}

void TestMotorVelocityStep(Motor &motor, Encoder &encoder, PidController &pid, float dir, float u_ff)
{

    float setpoint = 0;

    if (curr_micros < 5e6) { setpoint = 0; }
    else if (curr_micros < 10e6) { setpoint = 0.4 * dir; }
    else if (curr_micros < 15e6) { setpoint = 0.8 * dir; }
    else if (curr_micros < 20e6){ setpoint = 0.4 * dir; }
    else { setpoint = 0; }

    // Process encoder measurement
    encoder.ProcessMeasurement();

    // Get the current motor velocity normalized by the maximum velocity
    float curr_vel = encoder.get_vel_filt_rps() /motor.no_load_rps_;

    // Compute controller command based on desired and current position
    float control = pid.ComputeControl(setpoint, curr_vel, u_ff);

    // Send controller command to motor
    motor.set_command(control);

    if (curr_micros - prev_print_micros >= print_period_micros)
    {
        Serial.print(2.0);
        Serial.print(" ");
        Serial.print(-2.0);
        Serial.print(" ");
        Serial.print(setpoint);
        Serial.print(" ");
        Serial.println(curr_vel);

        prev_print_micros = curr_micros;
    }
}

void TestMotorVelocitySine(Motor &motor, Encoder &encoder, PidController &pid, float dir, float do_ff)
{

    float sin_freq = 0.5; // (Hz)
    float setpoint = dir * 0.5 * sin(2.0 * PI * sin_freq * curr_micros / 1.0e6);

    if (curr_micros < 4e6) { setpoint = 0; }
    else if (curr_micros > 12e6) { setpoint = 0; }

    // Process encoder measurement
    encoder.ProcessMeasurement();

    // Get the current motor velocity normalized by the maximum velocity
    float curr_vel = encoder.get_vel_filt_rps() /motor.no_load_rps_;

    // Compute controller command based on desired and current position
    float u_ff = do_ff * setpoint;
    float control = pid.ComputeControl(setpoint, curr_vel, u_ff);

    // Send controller command to motor
    motor.set_command(control);
//    motor.set_command(setpoint);

    if (curr_micros - prev_print_micros >= print_period_micros)
    {
        Serial.print(1.0);
        Serial.print(" ");
        Serial.print(-1.0);
//        Serial.print(" ");
//        Serial.print(encoder.count_ - encoder.prev_count_);
        Serial.print(" ");
        Serial.print(setpoint);
        Serial.print(" ");
        Serial.println(curr_vel);

        prev_print_micros = curr_micros;
    }
}

void TestMaxSpeed(Motor &motor, Encoder &encoder, float dir)
{


    encoder.ProcessMeasurement();

    if (curr_micros < 6e6)
    {
      
        motor.set_command(dir);
        
        if (curr_micros - prev_print_micros >= print_period_micros)
        {
//            Serial.print("Velocity (cps) = ");
//            Serial.println(encoder.vel_cps_);
//            Serial.print("Velocity (rps) = ");
            Serial.println(encoder.get_vel_filt_rps());
            prev_print_micros = curr_micros;
        }
    }
    else { motor.set_command(0.0); }
}

void TestDiffSteerForward(DiffSteer &robot)
{
    if (curr_micros < 4e6) { robot.Drive(0, 0); }
    else if (curr_micros < 6e6) { robot.Drive(robot.max_v_/4, 0); } // Forward
    else if (curr_micros < 8e6) { robot.Drive(robot.max_v_/2, 0); } // Forward
    else if (curr_micros < 10e6) { robot.Drive(robot.max_v_/4, 0); } // Forward
    else if (curr_micros < 11e6) { robot.Drive(0, 0); } // Stop
    else if (curr_micros < 13e6) { robot.Drive(-robot.max_v_/4, 0); } // Backward
    else if (curr_micros < 15e6) { robot.Drive(-robot.max_v_/2, 0); } // Backward
    else if (curr_micros < 17e6) { robot.Drive(-robot.max_v_/4, 0); } // Backward
    else { robot.Drive(0, 0); }
}

void TestDiffSteerRotate(DiffSteer &robot)
{
    if (curr_micros < 4e6) { robot.Drive(0, 0); }
    else if (curr_micros < 6e6) { robot.Drive(0, robot.max_w_/4); } // Forward
    else if (curr_micros < 8e6) { robot.Drive(0, robot.max_w_/2); } // Forward
    else if (curr_micros < 10e6) { robot.Drive(0, robot.max_w_/4); } // Forward
    else if (curr_micros < 11e6) { robot.Drive(0, 0); } // Stop
    else if (curr_micros < 13e6) { robot.Drive(0, -robot.max_w_/4); } // Backward
    else if (curr_micros < 15e6) { robot.Drive(0, -robot.max_w_/2); } // Backward
    else if (curr_micros < 17e6) { robot.Drive(0, -robot.max_w_/4); } // Backward
    else { robot.Drive(0, 0); }
}

void TestDiffSteerCircle(DiffSteer &robot, float radius, float dir)
{
    float v = robot.max_v_/3;
    float w = v / radius * dir;

    if (curr_micros < 4e6) { robot.Drive(0, 0); }
    else if (curr_micros < 16e6) { robot.Drive(v, w); }
    else { robot.Drive(0, 0); }

}

}


#endif
