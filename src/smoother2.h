
#ifndef SMOOTH2_H
#define SMOOTH2_H


#include <fstream>
#include <vector>
#include <math.h>

/*
 * This class performs moving average filtering
 *
 */
class Smoother2{

private:
    std::vector<float> _rbuf_data; // rotary buffer
    std::vector<float> _rbuf_time; // rotary buffer
	int _kernelsize; 	// filter kernel width
    int _rotater_data;		//pointer to current sample in rotary buffer
    int _rotater_time;		//pointer to current sample in rotary buffer
	float _runner; // current filter output value
    float _diff_data;
    float _derivative;
    float _derivative_filtered;
    float _filt_rate;
    bool _ready = false;

public:

    void init(int width, float filt_rate);
    float addSample(float data, float time);
    void reset(void);

};

#endif
