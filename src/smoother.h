
#ifndef SMOOTH_H
#define SMOOTH_H


#include <fstream>
#include <vector>
#include <math.h>

/*
 * This class performs moving average filtering
 *
 */
class Smoother{

private:
	std::vector<float> _rbuf; // rotary buffer
	int _kernelsize; 	// filter kernel width
	int _rotater;		//pointer to current sample in rotary buffer
	float _runner; // current filter output value

public:

	void init(int width);
	float addSample(float sample);

};

#endif
