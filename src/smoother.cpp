#include "smoother.h"

  void Smoother::init (int width ) {
  	_kernelsize = width;	
	_rbuf.resize(_kernelsize+1);
	_rotater = 0;
	_runner = 0;

	for (int i =0; i<= _kernelsize; i++) {
		_rbuf.at(i) = 0;
	}

  }


float Smoother::addSample(float sample) {
	//performs online smoothing filter

	if (isnan(sample)) // fixes nan, which forever destroy the output
		sample = 0;
    if (_kernelsize==1) { // disable smoothing... to be sure:
        return sample;
    }



	_rbuf.at(_rotater) = sample; // overwrite oldest sample in the roundtrip buffer
	_rotater = (_rotater+1) % (_kernelsize+1); //update pointer to buffer
	_runner = _runner + sample - _rbuf.at(_rotater); //add new sample, subtract the new oldest sample 

	return _runner /_kernelsize;

}
