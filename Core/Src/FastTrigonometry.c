/*
 * sincos.c
 *
 *  Created on: Mar 28, 2022
 *      Author: maxborglowe
 */

#include <FastTrigonometry.h>
#
#include "def.h"

float cossin_table[MAX_CIRCLE_ANGLE];    // Declare table of fast cosinus and sinus

void FastTrigonometry_buildTable() {
	long i;
	for (i = 0; i < MAX_CIRCLE_ANGLE; i++) {
		cossin_table[i] = (float) sin((double) i * _PI / HALF_MAX_CIRCLE_ANGLE);
	}
}

/**
 * @brief Fast cosine approximation using Maclaurin series expansion.
 * @param x: input angle in radians
 */
float FastTrigonometry_cos(float x) {
	float f = x * HALF_MAX_CIRCLE_ANGLE / _PI;
	int i = (int) f;

	if (i < 0) {
		return cossin_table[((-i) + QUARTER_MAX_CIRCLE_ANGLE)
				& MASK_MAX_CIRCLE_ANGLE];
	}
	else {
		return cossin_table[(i + QUARTER_MAX_CIRCLE_ANGLE)
				& MASK_MAX_CIRCLE_ANGLE];
	}
}

float FastTrigonometry_sin(float x) {
	float f = x * HALF_MAX_CIRCLE_ANGLE / _PI;
	int i = (int) f;
	if (i < 0) {
		return cossin_table[(-((-i) & MASK_MAX_CIRCLE_ANGLE))
				+ MAX_CIRCLE_ANGLE];
	}
	else {
		return cossin_table[i & MASK_MAX_CIRCLE_ANGLE];
	}
}


/**
 * @brief Fast arctan approximation function.
 * @param y: numerator
 * @param x: denominator
 * Source: https://www.dsprelated.com/showarticle/1052.php
 */
float FastTrigonometry_atan2(float y, float x) {
	const float n1 = 0.97239411f;
	const float n2 = -0.19194795f;
	float result = 0.0f;
	if (x != 0.0f) {
		const union {
			float flVal;
			uint32_t nVal;
		} tYSign = { y };

		const union {
			float flVal;
			uint32_t nVal;
		} tXSign = { x };

		if (fabsf(x) >= fabsf(y)) {
			union {
				float flVal;
				uint32_t nVal;
			} tOffset = { _PI };
			// Add or subtract PI based on y's sign.
			tOffset.nVal |= tYSign.nVal & 0x80000000u;
			// No offset if x is positive, so multiply by 0 or based on x's sign.
			tOffset.nVal *= tXSign.nVal >> 31;
			result = tOffset.flVal;
			const float z = y / x;
			result += (n1 + n2 * z * z) * z;
		}
		else    // Use atan(y/x) = pi/2 - atan(x/y) if |y/x| > 1.
		{
			union {
				float flVal;
				uint32_t nVal;
			} tOffset = { _PI_2 };

			// Add or subtract PI/2 based on y's sign.
			tOffset.nVal |= tYSign.nVal & 0x80000000u;
			result = tOffset.flVal;
			const float z = x / y;
			result -= (n1 + n2 * z * z) * z;
		}
	}
	else if (y > 0.0f) {
		result = _PI_2;
	}
	else if (y < 0.0f) {
		result = -_PI_2;
	}
	return result;
}

