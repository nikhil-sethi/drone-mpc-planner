
#ifndef SMOOTH_H
#define SMOOTH_H

#include <fstream>
#include <vector>
#include <cmath>

/*
 * This class performs moving average filtering
 *
 */
class Smoother
{

  private:
    std::vector<float> _rbuf; // rotary buffer
    uint16_t _kernelsize;          // filter kernel width
    uint16_t _rotater;             //pointer to current sample in rotary buffer
    float _runner;            // current filter output value
    bool _ready = false;

  public:
    void init(uint16_t width);
    void init(uint16_t width, float value);
    float addSample(float sample);
    float latest();
    void reset(void);
    void reset_to(float v);
    bool ready()
    {
        return _ready;
    }
    int kernselsize() { return _kernelsize;}
};

#endif
