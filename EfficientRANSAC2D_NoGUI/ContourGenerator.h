#pragma once

#include <vector>
#include <memory>
#include "PrimitiveShape.h"
#include "Util.h"


class ContourGenerator {
protected:
	ContourGenerator();
	~ContourGenerator();

public:
	static void generate(const util::Polygon& polygon, const std::vector<std::pair<int, std::shared_ptr<efficient_ransac::PrimitiveShape>>>& shapes, std::vector<cv::Point2f>& contour, float max_error, float angle_threshold);
};

