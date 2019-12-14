#pragma once
#include <fstream>
#include <vector>
#include <cmath>

namespace filtering {
class SmootherDerivative{

private:
    std::vector<float> _rbuf_data; // rotary buffer
    std::vector<float> _rbuf_time; // rotary buffer
    uint16_t _kernelsize; 	// filter kernel width
    uint16_t _rotater_data;		//pointer to current sample in rotary buffer
    uint16_t _rotater_time;		//pointer to current sample in rotary buffer
    float _runner; // current filter output value
    float _diff_data;
    float _derivative;
    float _derivative_filtered;
    float _filt_rate;
    bool _ready = false;

public:

    void init(uint16_t width, float filt_rate);
    float addSample(float data, float time);
    void reset(void);
    bool ready(){ return _ready;}

};
}
