#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

namespace Farmaid
{
    class PidController
    {
    public:
        PidController(float Kp, float Ki, float Kd, float Tf, float Ts)
            : Kp_(Kp), Ki_(Ki), Kd_(Kd), Tf_(Tf), Ts_(Ts),
              pos_deadband_(0), neg_deadband_(0), // control deadband
              prev_ym_(0), dymdt_(0), dymdt_filt_(0), prev_dymdt_filt_(0), ie_(0), e_(0), // states
              u_p_(0), u_i_(0), u_d_(0), u_(0) // control outputs
        {
          alpha_ = Ts_ / (Ts_ + Tf_); // derivate low pass filter coefficient
          // 0: one time step delay, 0-1: low-pass, 1: no filtering
          // alpha = Ts / (Ts + Tf) - typically around 0.1
          // alpha = 0: one time step delay, 0-1: low-pass, 1: no filtering
          // Tf should be about 10*Ts
          // higher alpha -> lower Tf -> higher cut-off freq. -> less filtering
        }

        float ComputeControl(float yr, float ym, float u_ff)
        {
            
            e_ = yr - ym; // Calculate error = reference - measurement
            dymdt_ = (ym - prev_ym_)/Ts_; // approximate derivative of measurement
            dymdt_filt_ = (1 - alpha_) * prev_dymdt_filt_ + alpha_ * dymdt_; // filter the derivative         
            ie_ = ie_ + e_ * Ts_; // Update integral error

            u_p_ = Kp_ * e_; // proportional control
            u_i_ = Ki_ * ie_; // integral control
            u_d_ = -Kd_ * dymdt_filt_; // derivative control on the measurement, not the reference

//            float u_ff = GetFeedforwardCtrl(yr);
            u_ = u_p_ + u_i_ + u_d_ + u_ff; // sum feedback and feedforward control
            u_ = GetCtrlWithDeadband(u_); // suppress control if smaller than deadband

            prev_ym_ = ym;
            prev_dymdt_filt_ = dymdt_filt_;

            return u_;
        }

        float GetCtrlWithDeadband(float u)
        {
            float u_db = u; 
            if (u > 0 && u < pos_deadband_)
            {
                u_db = 0;
            }
            else if (u < 0 && u > neg_deadband_)
            {
                u_db = 0;
            }
            return u_db;
        }

        void set_gains(float Kp, float Ki, float Kd, float Tf)
        {
            Kp_ = Kp;
            Ki_ = Ki;
            Kd_ = Kd;
            Tf_ = Tf;
            alpha_ = Ts_ / (Ts_ + Tf_); // derivate low pass filter coefficient
        }

        void set_deadband(float pos_deadband, float neg_deadband)
        {
          pos_deadband_ = pos_deadband;
          neg_deadband_ = neg_deadband;
        }

        void Reset()
        {
            e_ = 0;
            ie_ = 0;
//            ymf_ = 0;
//            ymf_prev_ = 0;
        }

        float u_p_; // proportional component of control command
        float u_i_; // integral component of control command
        float u_d_; // derivative component of control command
        float u_; // total control command

    private:
        float Kp_; // proportional gain
        float Ki_; // integral gain
        float Kd_; // derivative gain
        float Ts_; // sample time [sec]
        float Tf_; // derivate low pass filter time constant
        float alpha_; // derivative low pass filter coefficient
        float pos_deadband_; // positive control deadband
        float neg_deadband_; // negative control deadband

        float prev_ym_;
        float dymdt_;
        float dymdt_filt_;
        float prev_dymdt_filt_;
        float ie_; // integral of the error multiplied by sample period
        float e_;
    };
};

#endif
