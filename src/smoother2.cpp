#include "smoother2.h"

void Smoother2::init (int width, float filt_rate ) {
    _kernelsize = width;
    _filt_rate = filt_rate;
    _rbuf_data.resize(_kernelsize+1);
    _rbuf_time.resize(_kernelsize+1);
    _rotater_data = 0;
    _rotater_time = 0;
    _runner = 0;
    _diff_data = 0;
    _derivative_filtered = 0;

    for (int i =0; i<=_kernelsize; i++) {
        _rbuf_data.at(i) = 0;
        _rbuf_time.at(i) = 0;
    }
    //_rbuf_data.at(_kernelsize) = 0;

}

void Smoother2::reset() {
    _rotater_data = 0;
    _rotater_time = 0;
    for (int i =0; i<= _kernelsize; i++) {
        _rbuf_data.at(i) = 0;
        _rbuf_time.at(i) = 0;
    }
    //_rbuf_data.at(_kernelsize) = 0;
    _runner = 0;
    _derivative_filtered = 0;
    _ready = false;
}

float Smoother2::addSample(float data, float time) {
    //performs online smoothing filter

    if (isnanf(data)) // fixes nan, which forever destroy the output
        data = 0;
    if (_kernelsize==1) { // disable smoothing... to be sure:
        return data;
    }

    _rbuf_data.at(_rotater_data) = data; // overwrite oldest sample in the roundtrip buffer
    _rbuf_time.at(_rotater_time) = time; // overwrite oldest sample in the roundtrip buffer

    _rotater_data = (_rotater_data+1) % (_kernelsize+1); //update pointer to buffer
    _rotater_time = (_rotater_time+1) % (_kernelsize+1); //update pointer to buffer

    // keep track of dt
    _runner = _runner + time - _rbuf_time.at(_rotater_time); //add new sample, subtract the new oldest sample

    // calculate data difference
    _diff_data = data - _rbuf_data.at(_rotater_data);

    // determine derivative: d/dt
    _derivative = _diff_data/_runner;

    // filter the derivative
    _derivative_filtered = _derivative*_filt_rate + _derivative_filtered*(1.0f-_filt_rate);

    if (_ready) {

        return _derivative_filtered;

    }
    else {

        if (_rotater_data==0) { // the buffer is filled, set filter to current value
            _ready = true;
            _derivative_filtered = _derivative;
        }

        return 0;

    }
}
