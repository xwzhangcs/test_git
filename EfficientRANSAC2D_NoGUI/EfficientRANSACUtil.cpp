#include "EfficientRANSACUtil.h"

namespace efficient_ransac {

	/**
	* return angle difference between two angles in range [0, PI/2].
	*/
	float angle_difference(float x, float xi) {
		x = regularize_angle_PI(x);
		xi = regularize_angle_PI(xi);

		if (std::abs(x - xi) <= CV_PI * 0.5) return std::abs(x - xi);
		else return CV_PI - std::abs(x - xi);
	}

	/**
	* return the normalized angle in range [0, PI].
	*/
	float regularize_angle_PI(float x) {
		if (x < 0) {
			x += CV_PI * (int)(-x / CV_PI + 1);
		}
		else {
			x -= CV_PI * (int)(x / CV_PI);
		}

		return x;
	}

}
