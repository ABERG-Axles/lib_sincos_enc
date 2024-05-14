/*
 * utils.c
 *
 *  Created on: May 13, 2024
 *      Author: t.shypytiak
 */

#include "enc_utils.h"

//std
#include <math.h>

float utils_fast_atan2(float y, float x) {
	float abs_y = fabsf( y ) + 1e-20f; // kludge to prevent 0/0 condition
	float angle;

	if (x >= 0) {
		float r = (x - abs_y) / (x + abs_y);
		float rsq = r * r;
		angle = (float)( ((0.1963 * rsq) - 0.9817) * r + ( M_PI / 4.0 ) );
	} else {
		float r = (x + abs_y) / (abs_y - x);
		float rsq = r * r;
		angle = (float)( ((0.1963f * rsq) - 0.9817f) * r + (3.0f * M_PI / 4.0f) );
	}

	UTILS_NAN_ZERO(angle);

	if (y < 0) {
		return(-angle);
	} else {
		return(angle);
	}
}
