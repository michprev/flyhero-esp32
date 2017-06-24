/*
 * Biquad_Filter.cpp
 *
 *  Created on: 24. 6. 2017
 *      Author: michp
 */

#include <Biquad_Filter.h>

namespace flyhero {

Biquad_Filter::Biquad_Filter(Filter_Type type, float sample_frequency, float cut_frequency) {
	double K = std::tan(this->PI * cut_frequency / sample_frequency);
	double Q = 1.0 / std::sqrt(2); // let Q be 1 / sqrt(2) for Butterworth

	double norm;

	switch (type) {
	case FILTER_LOW_PASS:
		norm = 1.0 / (1 + K / Q + K * K);

		this->a0 = K * K * norm;
		this->a1 = 2 * this->a0;
		this->a2 = this->a0;
		this->b1 = 2 * (K * K - 1) * norm;
		this->b2 = (1 - K / Q + K * K) * norm;

		break;
	case FILTER_NOTCH:
		norm = 1.0 / (1 + K / Q + K * K);

		this->a0 = (1 + K * K) * norm;
		this->a1 = 2 * (K * K - 1) * norm;
		this->a2 = this->a0;
		this->b1 = this->a1;
		this->b2 = (1 - K / Q + K * K) * norm;

		break;
	}

	this->z1 = 0;
	this->z2 = 0;
}

} /* namespace flyhero */
