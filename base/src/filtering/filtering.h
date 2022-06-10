#pragma once
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include "third_party/tinyxml/XMLSerialization.h"
#include <iostream>
#include <fstream>
#include "smoother.h"

namespace filtering {

class Tf_PT1_f {
private:
    float y;
    float sample_time;
    float time_factor;
    float K;
public:
    void init(float init_sample_time, float init_K, float T) {
        sample_time = init_sample_time;
        change_dynamic(T);
        K = init_K;
        y = 0;
    }

    void reset(float y0) {
        y = y0;
    }

    void change_dynamic(float T) {
        if (T > 0)
            time_factor = 1 - exp(-sample_time / T);
        else
            time_factor = 1.f;
    }

    float new_sample(float input) {
        y = time_factor * (K * input - y) + y;
        return y;
    }

    float current_output() {
        return y;
    }
};

class Tf_PT1_3f {
private:
    cv::Point3f y;
    float sample_time;
    float time_factor;
    float K;
public:
    void init(float init_sample_time, float init_K, float T) {
        sample_time = init_sample_time;
        change_dynamic(T);
        K = init_K;
        y = cv::Point3f(0, 0, 0);
    }

    void reset(cv::Point3f y0) {
        y = y0;
    }

    void change_dynamic(float T) {
        if (T > 0)
            time_factor = 1 - exp(-sample_time / T);
        else
            time_factor = 1.f;
    }

    cv::Point3f new_sample(cv::Point3f input) {
        y = time_factor * (K * input - y) + y;
        return y;
    }

    cv::Point3f current_output() {
        return y;
    }
};

class Tf_PT2_f {
private:
    float yk, yk1, yk2;
    float sample_time;
    float K, T1, T2;

public:
    void init(float init_sample_time, float init_K, float init_T1, float init_T2) {
        sample_time = init_sample_time;
        K = init_K;
        T1 = init_T1;
        T2 = init_T2;
    }

    float new_sample(float uk) {
        yk2 = yk1;
        yk1 = yk;
        yk = uk * K * powf(sample_time, 2) + yk1 * (2 * T1 * T2 + (T1 + T2) * sample_time) - yk2 * T1 * T2;
        yk /= T1 * T2 + (T1 + T2) * sample_time + powf(sample_time, 2);

        return yk;
    }

    void dynamic(float new_T1, float new_T2) {
        T1 = new_T1;
        T2 = new_T2;
    }

    void internal_states(float init_yk1, float init_yk2) {
        yk = init_yk1;
        yk1 = init_yk2;
    }

    float current_output() {
        return yk;
    }
};

class Tf_PT2_3f {
private:
    cv::Point3f yk, yk1, yk2;
    float sample_time;
    float K, T1, T2;

public:
    void init(float init_sample_time, float init_K, float init_T1, float init_T2) {
        sample_time = init_sample_time;
        K = init_K;
        T1 = init_T1;
        T2 = init_T2;
    }

    cv::Point3f new_sample(cv::Point3f uk) {
        yk2 = yk1;
        yk1 = yk;
        yk = uk * K * powf(sample_time, 2) + yk1 * (2 * T1 * T2 + (T1 + T2) * sample_time) - yk2 * T1 * T2;
        yk /= T1 * T2 + (T1 + T2) * sample_time + powf(sample_time, 2);

        return yk;
    }

    void dynamic(float new_T1, float new_T2) {
        T1 = new_T1;
        T2 = new_T2;
    }

    void internal_states(cv::Point3f init_yk1, cv::Point3f init_yk2) {
        yk = init_yk1;
        yk1 = init_yk2;
    }

    cv::Point3f current_output() {
        return yk;
    }
};

class Tf_D_f {
private:
    float y;
    float u_prev;
    float sample_time;

public:
    void init(float init_sample_time) {
        sample_time = init_sample_time;
        u_prev = 0;
    }

    void preset(float input0) {
        u_prev = input0;
    }

    float new_sample(float input) {
        y = (input - u_prev) / sample_time;
        u_prev = input;
        return y;
    }

    float current_output() {
        return y;
    }
};

class Tf_Tt_3f {
private:
    std::vector<cv::Point3f> buffer;
    uint buffer_ptr = 0;
    uint buffer_size;

public:
    void init(float init_sample_time, float delay_time) {
        uint discrete_delay = static_cast<uint>(delay_time / init_sample_time);
        buffer_size = discrete_delay + 1;
        buffer.resize(buffer_size);
        this->preset({0});
    }

    void preset(cv::Point3f init_state) {
        for (size_t i = 0; i < buffer_size; i++) {
            buffer.at(i) = init_state;
        }
    }

    cv::Point3f new_sample(cv::Point3f input) {
        cv::Point3f output = buffer.at(buffer_ptr);
        buffer.at(buffer_ptr) = input;
        buffer_ptr++;
        buffer_ptr %= buffer_size;

        return output;
    }
};

class hold_filter {
private:
    double hold_time;
    double time_last_non_zero;
    int hold_value;

public:
    void init(double holdtime) {
        hold_time = holdtime;
    }
    int update(int new_sample, double time) {

        if (new_sample) {
            time_last_non_zero = time;
            hold_value = new_sample;
        }

        if ((time - time_last_non_zero) > hold_time) {
            hold_value = 0;
            return hold_value;
        }
        else
            return hold_value;
    }

    int output() {
        return hold_value;
    }
};
}
