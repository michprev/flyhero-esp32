/*
 * Biquad_Filter.h
 *
 *  Created on: 24. 6. 2017
 *      Author: michp
 */

#ifndef BIQUAD_FILTER_H_
#define BIQUAD_FILTER_H_

#include <cmath>

namespace flyhero {

// Many thanks to http://www.earlevel.com/main/2012/11/26/biquad-c-source-code/

class Biquad_Filter {
private:
	const double PI = 3.14159265358979323846;

	float a0, a1, a2;
	float b1, b2;
	float z1, z2;
public:
	enum Filter_Type { FILTER_LOW_PASS, FILTER_NOTCH };

	Biquad_Filter(Filter_Type type, float sample_frequency, float cut_frequency);

	inline float Apply_Filter(float value);
};

// 10 us
// https://en.wikipedia.org/wiki/Digital_biquad_filter - Transposed direct forms
float Biquad_Filter::Apply_Filter(float value) {
	float ret = value * this->a0 + this->z1;
	this->z1 = value * this->a1 + this->z2 - this->b1 * ret;
	this->z2 = value * this->a2 - this->b2 * ret;

	return ret;
}

} /* namespace flyhero */

#endif /* BIQUAD_FILTER_H_ */
