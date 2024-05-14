/*
 * utils.h
 *
 *  Created on: May 13, 2024
 *      Author: t.shypytiak
 */

#ifndef APPLICATION_USER_UTILS_H_
#define APPLICATION_USER_UTILS_H_
//#include <math.h>
/**
 * A simple low pass filter.
 *
 * @param value
 * The filtered value.
 *
 * @param sample
 * Next sample.
 *
 * @param filter_constant
 * Filter constant. Range 0.0 to 1.0, where 1.0 gives the unfiltered value.
 */
#define LP_FAST(value, sample, filter_constant)	(value -= (filter_constant) * ((value) - (sample)))

#define SQ(x)				((x) * (x))

#ifndef M_PI
	#define M_PI		3.14159265358979323846
#endif

#define RAD2DEG( rad ) ( ( rad ) * ( float )( 180.0 / M_PI ) )
#define DEG2RAD( deg ) ( ( deg ) * ( float )( M_PI / 180.0 ) )

#define UTILS_IS_NAN(x)		((x) != (x))
#define UTILS_NAN_ZERO(x)	(x = UTILS_IS_NAN(x) ? 0.0 : x)

/**
 *
 * http://www.dspguru.com/dsp/tricks/fixed-point-atan2-with-self-normalization
 * @return The angle in radians
 */

float utils_fast_atan2( float y, float x );

#endif /* APPLICATION_USER_UTILS_H_ */
