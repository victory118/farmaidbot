/**
 * @file ls7184_encoder.h
 * @brief Quadrature encoder counter driver for the LSI-LS7184
 * @author Victor Yu
 */

#ifndef LS7184_ENCODER_H
#define LS7184_ENCODER_H

namespace Farmaid
{
    class Encoder
    {
    public:

        int count_;
        int prev_count_;
        float vel_cps_; // velocity [encoder counts/s]
        float vel_filt_cps_; // filtered velocity [encoder counts/s]
        float prev_vel_filt_cps_;

        const byte clk_pin_;
        const byte dir_pin_;
        const float counts_per_rev_; // encoder counts per revolution
        const float Ts_; // time period between sampling encoder measurements [sec]
        
        Encoder(byte clk_pin, byte dir_pin, float counts_per_rev, float Ts, float Tf, volatile int *count_ptr)
            : clk_pin_(clk_pin), dir_pin_(dir_pin),
              counts_per_rev_(counts_per_rev), Ts_(Ts), Tf_(Tf_),
              count_ptr_(count_ptr),
              count_(0), prev_count_(0),
              vel_cps_(0), vel_filt_cps_(0), prev_vel_filt_cps_(0)                        
        {
            pinMode(clk_pin_, INPUT);
            pinMode(dir_pin_, INPUT);
            alpha_ = Ts_ / (Ts_ + Tf_);
        }

        void ProcessMeasurement()
        {
            // Store the previous count and read the new count
            prev_count_ = count_;
            count_ = *count_ptr_;
            
            vel_cps_ = (float)(count_ - prev_count_) / Ts_;

            // Approximate the velocity
            vel_filt_cps_ = (1 - alpha_) * prev_vel_filt_cps_ + alpha_ * vel_cps_;

            prev_vel_filt_cps_ = vel_filt_cps_;
        }

        void Reset()
        {
            count_ = 0;
            prev_count_ = 0;
            vel_cps_ = 0;
            vel_filt_cps_ = 0;
            prev_vel_filt_cps_ = 0;
        }

        float get_delta_rad() { return (float)(count_ - prev_count_) / counts_per_rev_ * 2.0 * PI; }
        float get_vel_rps() { return vel_cps_ / counts_per_rev_ * 2.0 * PI; }
        float get_vel_filt_rps() { return vel_filt_cps_ / counts_per_rev_ * 2.0 * PI; }

    private:
        const float Tf_; // first order low-pass filter time constant
        float alpha_; // first order low-pass filter coefficient

        volatile int *count_ptr_; // pointer to the global variable that holds the encoder counts
    };
};

#endif
