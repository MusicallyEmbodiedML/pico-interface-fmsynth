#ifndef __ONE_POLE_SMOOTHER_HPP__
#define __ONE_POLE_SMOOTHER_HPP__

template<size_t n_channels>
class OnePoleSmoother {
 public:
    OnePoleSmoother(float time_ms, float sample_rate) :
        sample_rate_(sample_rate),
        y_ { 0 } {
        SetTimeMs(time_ms);
    }
    void SetTimeMs(float time_ms) {
        //b1_ = std::exp(
        //    std::log(0.01) /
        //    time_ms * sample_rate_ * 0.001
        //);
        b1_ = std::pow(0.1, 1.f/ (time_ms * 0.001 * sample_rate_));
    }
    __attribute__((always_inline)) void Process(const float * x_ptr, float *y_ptr) {
        float *y2_ptr = y_;
        for (unsigned int c = 0; c < n_channels; c++) {
            const float x = *x_ptr;
            *y2_ptr = *y_ptr = x + b1_ * (*y2_ptr - x);
            ++x_ptr;
            ++y_ptr;
            ++y2_ptr;
        }
    }

 protected:
    float sample_rate_;
    float b1_;
    float y_[n_channels];
};

#endif  // __ONE_POLE_SMOOTHER_HPP__