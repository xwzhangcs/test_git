#pragma once

#include <vector>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace efficient_ransac {

	float angle_difference(float x, float xi);
	float regularize_angle_PI(float x);
	

}
