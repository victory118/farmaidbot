#ifndef ROBOT_4WD_H
#define ROBOT_4WD_H

#include "diff_steer.h"

namespace Farmaid
{  
    class Robot4WD : public DiffSteer
    {
    public:
        Robot4WD (Encoder fleft_encoder, Encoder fright_encoder,
              Encoder rleft_encoder, Encoder rright_encoder,
              Motor fleft_motor, Motor fright_motor,
              Motor rleft_motor, Motor rright_motor,
              PidController fleft_pid, PidController fright_pid,
              PidController rleft_pid, PidController rright_pid,
              float wheelbase, float wheel_radius)
        : fleft_encoder_(fleft_encoder), fright_encoder_(fright_encoder),
          rleft_encoder_(rleft_encoder), rright_encoder_(rright_encoder),
          fleft_motor_(fleft_motor), fright_motor_(fright_motor),
          rleft_motor_(rleft_motor), rright_motor_(rright_motor),
          fleft_pid_(fleft_pid), fright_pid_(fright_pid),
          rleft_pid_(rleft_pid), rright_pid_(rright_pid),
          DiffSteer(wheelbase, wheel_radius)
        {
            max_v_ = min(min(fleft_motor_.no_load_rps_, fright_motor_.no_load_rps_),
                           min(rleft_motor_.no_load_rps_, rright_motor_.no_load_rps_)) * wheel_radius_;
            max_w_ = max_v_ / (wheelbase_ / 2);
        }

        void Drive(float v, float w)
        {
            // Process encoder measurements
            fleft_encoder_.ProcessMeasurement();
            fright_encoder_.ProcessMeasurement();
            rleft_encoder_.ProcessMeasurement();
            rright_encoder_.ProcessMeasurement();

            // Calculate the current wheel velocities
            float wv_fl = fleft_encoder_.get_vel_rps() * wheel_radius_;
            float wv_fr = fright_encoder_.get_vel_rps() * wheel_radius_;
            float wv_rl = rleft_encoder_.get_vel_rps() * wheel_radius_;
            float wv_rr = rright_encoder_.get_vel_rps() * wheel_radius_;

            // Compute the desired wheel velocities
            DiffDriveWheelVel dd_wv = UniToDiff(v, w);

            // Compute controller commands based on desired and current wheel velocities
            float ctrl_fl = fleft_pid_.ComputeControl(dd_wv.left, wv_fl, dd_wv.left/max_v_);
            float ctrl_fr = fright_pid_.ComputeControl(dd_wv.right, wv_fr, dd_wv.right/max_v_);
            float ctrl_rl = rleft_pid_.ComputeControl(dd_wv.left, wv_rl, dd_wv.left/max_v_);
            float ctrl_rr = rright_pid_.ComputeControl(dd_wv.right, wv_rr, dd_wv.right/max_v_);

            // Send controller commands to each motor
            fleft_motor_.set_command(ctrl_fl);
            fright_motor_.set_command(ctrl_fr);
            rleft_motor_.set_command(ctrl_rl);
            rright_motor_.set_command(ctrl_rr);
        }

        void UpdateOdometry()
        {   
            // dphi_right/left: change in wheel angle (radians) since last measurement
            // Calculate the average of front and rear wheel measurement
            float dphi_right = (fright_encoder_.get_delta_rad() + rright_encoder_.get_delta_rad()) / 2.0;
            float dphi_left = (fleft_encoder_.get_delta_rad() + rleft_encoder_.get_delta_rad()) / 2.0;
            
            float dtheta = (wheel_radius_/wheelbase_) * (dphi_right - dphi_left);
            float dx = (wheel_radius_/2) * (dphi_right + dphi_left) * cos(odom_.theta);
            float dy = (wheel_radius_/2) * (dphi_right + dphi_left) * sin(odom_.theta);
            odom_.pos_x += dx;
            odom_.pos_y += dy;
            odom_.theta += dtheta;
            odom_.theta = atan2(sin(odom_.theta),cos(odom_.theta));
            odom_.wheel_angle_left += dphi_left;
            odom_.wheel_angle_right += dphi_right;
        }
        
    private:
        Encoder fleft_encoder_;
        Encoder fright_encoder_;
        Encoder rleft_encoder_;
        Encoder rright_encoder_;

        Motor fleft_motor_;
        Motor fright_motor_;
        Motor rleft_motor_;
        Motor rright_motor_;

        PidController fleft_pid_;
        PidController fright_pid_;
        PidController rleft_pid_;
        PidController rright_pid_;
        
    };
};

#endif
