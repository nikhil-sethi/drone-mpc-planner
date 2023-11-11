#include "smoother.h"
namespace filtering {
void Smoother::init(uint16_t width, float value) {
    _kernelsize = width;
    _rbuf.resize(_kernelsize + 1);
    _rotater = 0;
    _runner = value * _kernelsize;
    for (uint16_t i = 0; i < _kernelsize + 1; i++) {
        _rbuf.at(i) = value;
    }
    _ready = true;
}

void Smoother::init(uint16_t width) {
    init(width, 0);
    _ready = false;
}

void Smoother::reset() {
    init(_kernelsize);
}

void Smoother::change_width(int new_width) {
    float avg = latest();
    init(new_width, avg);
}

float Smoother::addSample(float sample) {
    if (isnanf(sample)) // fixes nan, which forever destroy the output otherwise
        sample = 0;
    if (_kernelsize == 1) {   // disable smoothing...
        _ready = true;
        _runner = sample;
        return sample;
    }

    _rbuf.at(_rotater) = sample;                     // overwrite oldest sample in the roundtrip buffer
    _rotater = (_rotater + 1) % (_kernelsize + 1);   //update pointer to buffer
    _runner = _runner + sample - _rbuf.at(_rotater); //add new sample, subtract the new oldest sample

    if (!_ready) {   // check if completely filled
        if (!_rotater)
            _ready = true;
        else
            return _runner / _rotater; // if not filled completely, return average over the amount of added data (the rest of the filter is initialised to zero)
    }

    return _runner / _kernelsize;
}

float Smoother::latest() {
    if (!_ready)
        return _runner / _rotater;
    else
        return _runner / _kernelsize;
}
}