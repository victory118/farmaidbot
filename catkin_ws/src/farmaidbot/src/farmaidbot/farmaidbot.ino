#include <Arduino.h>
#include "mega4wd.h" // initialize motor, encoder, and PID parameters and encoder interrupt callback functions
#include "test.h"
#include "serial_comm.h"

Farmaid::SerialComm serial_comm; // serial communication object
unsigned long curr_micros;
unsigned long prev_ctrl_micros = 0;
unsigned long prev_print_micros = 0;

void setup() {

    Serial.begin(115200); // initialize serial communication
    
    // Attach encoder interrupts
    attachInterrupt(4, FLeftEncoderInterrupt, RISING);
    attachInterrupt(5, FRightEncoderInterrupt, RISING);
    attachInterrupt(0, RRightEncoderInterrupt, RISING);
    attachInterrupt(1, RLeftEncoderInterrupt, RISING);

    // Necessary for encoder interrupts to initialize
    delay(1000);
}

void loop() {

    curr_micros = micros();

    if (curr_micros - prev_ctrl_micros >= ctrl_period_micros) {
        
        // Update the odometry
        robot.UpdateOdometry();

        // Send odometry through serial communication
        serial_comm.send(robot.odom_);

        // Receive velocity commands through serial communication
        serial_comm.receiveSerialData();

        // Send the velocity and angular velocity commands to the controller
//        Farmaid::TestDiffSteerCircle(robot, 0.3, 1);
        robot.Drive(serial_comm.v_des_, serial_comm.w_des_); // drive the robot

        // Test encoders
//        ReadEncoders(); // Passed!
//        Farmaid::TestEncoderClass(fleft_encoder, fright_encoder); // Passed!
//        Farmaid::TestEncoderClass(rleft_encoder, rright_encoder); // Passed!
        
        // Test motor open loop
//        Farmaid::TestMotorOpenLoop(fleft_motor, 1); // Passed
//        Farmaid::TestMotorOpenLoop(fleft_motor, -1); // Passed
//        Farmaid::TestMotorOpenLoop(fright_motor, 1); // Passed
//        Farmaid::TestMotorOpenLoop(fright_motor, -1); // Passed
//        Farmaid::TestMotorOpenLoop(rleft_motor, 1); // Passed/
//        Farmaid::TestMotorOpenLoop(rleft_motor, -1); // Passed
//        Farmaid::TestMotorOpenLoop(rright_motor, 1); // Passed
//        Farmaid::TestMotorOpenLoop(rright_motor, -1); // Passed

        // Find deadband
//        fleft_motor.set_command(0.04);
//        fleft_motor.set_command(-0.04);
//        fright_motor.set_command(0.04);
//        fright_motor.set_command(-0.04);
//        rright_motor.set_command(0.04);
//        rright_motor.set_command(-0.04);
//        rleft_motor.set_command(0.04);
//        rleft_motor.set_command(-0.04);

        // Motor position control
//        fleft_pid.set_gains(7.5, 0.5, 0.0, 0.02);
//        fleft_pid.set_deadband(0.04,-0.04);
//        Farmaid::TestMotorPositionControl(fleft_motor, fleft_encoder, fleft_pid, 1); // Passed
//        Farmaid::TestMotorPositionControl(fleft_motor, fleft_encoder, fleft_pid, -1); // Passed
//        fright_pid.set_gains(7.5, 0.5, 0.0, 0.02);
//        fright_pid.set_deadband(0.04,-0.04);
//        Farmaid::TestMotorPositionControl(fright_motor, fright_encoder, fright_pid, 1); // Passed
//        Farmaid::TestMotorPositionControl(fright_motor, fright_encoder, fright_pid, -1); // Passed
//        rleft_pid.set_gains(7.5, 0.5, 0.0, 0.02);
//        rleft_pid.set_deadband(0.04,-0.04);
//        Farmaid::TestMotorPositionControl(rleft_motor, rleft_encoder, rleft_pid, 1); // Passed
//        Farmaid::TestMotorPositionControl(rleft_motor, rleft_encoder, rleft_pid, -1); // Passed
//        rright_pid.set_gains(7.5, 0.5, 0.0, 0.02);
//        rright_pid.set_deadband(0.04,-0.04);
//        Farmaid::TestMotorPositionControl(rright_motor, rright_encoder, rright_pid, 1); // Passed
//        Farmaid::TestMotorPositionControl(rright_motor, rright_encoder, rright_pid, -1); // Passed

        // Find maximum speed of each motor to add feedforward to PID controller

//        Farmaid::TestMaxSpeed(fleft_motor, fleft_encoder, 1); // 8.39 rad/s
//        Farmaid::TestMaxSpeed(fleft_motor, fleft_encoder, -1); // -8.39 rad/s
//        Farmaid::TestMaxSpeed(fright_motor, fright_encoder, 1); // 8.39 rad/s/
//        Farmaid::TestMaxSpeed(fright_motor, fright_encoder, -1); // 8.39 rad/s

//        Farmaid::TestMaxSpeed(rleft_motor, rleft_encoder, 1); 8.39
//        Farmaid::TestMaxSpeed(rleft_motor, rleft_encoder, -1);
//        Farmaid::TestMaxSpeed(rright_motor, rright_encoder, 1);
//        Farmaid::TestMaxSpeed(rright_motor, rright_encoder, -1);

        // Sinusoidal velocity command with PID and feedforward
        
//        fleft_pid.set_gains(2.0, 0.0, 0.0, 10.0*ctrl_period);
//        Farmaid::TestMotorVelocitySine(fleft_motor, fleft_encoder, fleft_pid, 1, 1); // Passed
//        Farmaid::TestMotorVelocitySine(fleft_motor, fleft_encoder, fleft_pid, -1, 1); // Passed
//        fright_pid.set_gains(2.0, 0.0, 0.0, 10.0*ctrl_period);
//        Farmaid::TestMotorVelocitySine(fright_motor, fright_encoder, fright_pid, 1, 1); // Passed
//        Farmaid::TestMotorVelocitySine(fright_motor, fright_encoder, fright_pid, -1, 1); // Passed
//        rleft_pid.set_gains(2.0, 0.0, 0.0, 10.0*ctrl_period);
//        Farmaid::TestMotorVelocitySine(rleft_motor, rleft_encoder, rleft_pid, 1, 1); // Passed
//        Farmaid::TestMotorVelocitySine(rleft_motor, rleft_encoder, rleft_pid, -1, 1); // Passed
//        rright_pid.set_gains(2.0, 0.0, 0.0, 10.0*ctrl_period);
//        Farmaid::TestMotorVelocitySine(rright_motor, rright_encoder, rright_pid, 1, 1); // Passed
//        Farmaid::TestMotorVelocitySine(rright_motor, rright_encoder, rlright_pid, -1, 1); // Passed

        // Test robot drive method (PID gains and feedforward_flag set in Robot constructor)
//        Farmaid::TestDiffSteerForward(robot); // Passed!
//        Farmaid::TestDiffSteerRotate(robot);
//        Farmaid::TestDiffSteerCircle(robot, 0.3, 1);

        prev_ctrl_micros = curr_micros;
    }

}
