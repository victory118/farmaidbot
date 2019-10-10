#ifndef DIFF_STEER_H
#define DIFF_STEER_H

namespace Farmaid
{  
    struct DiffDriveWheelVel
    {
        // struct to hold left and right wheel velocity variables
        float left; // left wheel velocity [m/s]
        float right;// right wheel velocity [m/s]
    };

    struct Odometry2D
    {
        float pos_x;
        float pos_y;
        float theta;
        float wheel_angle_left;
        float wheel_angle_right;
    };
    
    class DiffSteer
    {
    public:
        DiffSteer(float wheelbase, float wheel_radius) : 
            wheelbase_(wheelbase), wheel_radius_(wheel_radius),
            max_v_(0), max_w_(0), ensure_w_(true),
            odom_{0, 0, 0, 0, 0}
        {
        }
        
        /**
         * @brief Converts unicycle velocity commands to motor velocity commands
         * @param vel: commanded forward velocity in m/s
         * ang_vel: commanded angular velocity in m/s
         */
        virtual void Drive(float v, float w) = 0;
        virtual void UpdateOdometry() = 0;

        DiffDriveWheelVel UniToDiff(float v, float w)
        {
            // This function ensures that ang_vel is respected as best as possible
            // by scaling vel.
            // vel - desired robot linear velocity [m/s]
            // ang_vel - desired robot angular velocity [rad/s]

            DiffDriveWheelVel dd_wv = {0.0, 0.0};

            if (!ensure_w_)
            {
                dd_wv.right = (v + w * wheelbase_ / 2.0);
                dd_wv.left = (v - w* wheelbase_ / 2.0);
                return dd_wv;
            }
            
            // 1. Limit v and w to be within the possible range
            float w_lim = max(min(w, max_w_), -max_w_);
            float v_lim = max(min(v, max_v_), -max_v_);

            // 2. Compute left and right wheel velocities required to achieve limited v and w
            float wv_lim_right = (v_lim + w_lim * wheelbase_ / 2.0);
            float wv_lim_left = (v_lim - w_lim * wheelbase_ / 2.0);

            // 3. Find max and min of the limited wheel velocities
            float wv_lim_max = max(wv_lim_right, wv_lim_left);
            float wv_lim_min = min(wv_lim_right, wv_lim_left);

            // 4. Shift limited wheel velocities if they exceed the maximum wheel velocity
            if (wv_lim_max > max_v_)
            {
                dd_wv.right = wv_lim_right - (wv_lim_max - max_v_);
                dd_wv.left = wv_lim_left - (wv_lim_max - max_v_);
            }
            else if (wv_lim_min < -max_v_) 
            {
                dd_wv.right = wv_lim_right - (wv_lim_min + max_v_);
                dd_wv.left = wv_lim_left - (wv_lim_min + max_v_);
            }
            else
            {
                dd_wv.right = wv_lim_right;
                dd_wv.left = wv_lim_left;
            }

            return dd_wv;
        }

        Odometry2D odom_;
        float max_v_; // maximum forward velocity of robot with no rotation [m/s]
        float max_w_; // maximum angular velocity of robot with pure rotation [rad/s]
        bool ensure_w_; // flag to constrain motor speeds if necessary to ensure angular velocity is satisfied
        
    protected:
        float wheelbase_;
        float wheel_radius_;

    };
};

#endif
