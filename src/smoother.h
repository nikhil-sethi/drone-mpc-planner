
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
    int _kernelsize;          // filter kernel width
    int _rotater;             //pointer to current sample in rotary buffer
    float _runner;            // current filter output value
    bool _ready = false;

  public:
    void init(int width);
    void init(int width, float value);
    float addSample(float sample);
    float get_latest();
    void reset(void);
    void reset_to(float v);
    bool get_ready()
    {
        return _ready;
    }
    int get_kernselsize() { return _kernelsize;}
};

#endif
