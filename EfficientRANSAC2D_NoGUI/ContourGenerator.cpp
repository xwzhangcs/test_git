#include "ContourGenerator.h"
#include "CurveDetector.h"
#include "LineDetector.h"

ContourGenerator::ContourGenerator() {
}

ContourGenerator::~ContourGenerator() {
}

void ContourGenerator::generate(const util::Polygon& polygon, const std::vector<std::pair<int, std::shared_ptr<efficient_ransac::PrimitiveShape>>>& shapes, std::vector<cv::Point2f>& contour, float max_error, float angle_threshold) {
	for (int j = 0; j < shapes.size(); j++) {
		if (efficient_ransac::Circle* circle = dynamic_cast<efficient_ransac::Circle*>(shapes[j].second.get())) {
			int num = circle->angleRange() / CV_PI * 180 / 10;
			contour.push_back(circle->startPoint());
			for (int k = 1; k < num; k++) {
				float angle = circle->startAngle() + circle->signedAngleRange() / num * k;
				contour.push_back(circle->center() + circle->radius() * cv::Point2f(std::cos(angle), std::sin(angle)));
			}
			contour.push_back(circle->endPoint());

			int j2 = (j + 1) % shapes.size();
			contour.push_back(shapes[j2].second->startPoint());
		}
		else if (efficient_ransac::Line* line = dynamic_cast<efficient_ransac::Line*>(shapes[j].second.get())) {
			int j2 = (j + 1) % shapes.size();
			if (efficient_ransac::Circle* circle2 = dynamic_cast<efficient_ransac::Circle*>(shapes[j2].second.get())) {
				contour.push_back(line->endPoint());
				continue;
			}
			else if (efficient_ransac::Line* line2 = dynamic_cast<efficient_ransac::Line*>(shapes[j2].second.get())) {
				/*
				// check if two adjacent shapes are almost collinear
				cv::Point2f dir1 = shapes[j].second->endPoint() - shapes[j].second->startPoint();
				cv::Point2f dir2 = shapes[j2].second->endPoint() - shapes[j2].second->startPoint();
				if (std::abs(dir1.dot(dir2) / cv::norm(dir1) / cv::norm(dir2)) >= std::cos(angle_threshold)) {
					// if two shapes are close enough, merge them
					if (line->distance(line2->startPoint()) < max_error) continue;
					else {
						// simpliy connect two shapes
						contour.push_back(line->endPoint());
						contour.push_back(line2->startPoint());
					}
				}

				// calculate the intersection between the current shape and the next one
				cv::Point2f int_pt;
				double tab, tcd;
				if (util::lineLineIntersection(shapes[j].second->startPoint(), shapes[j].second->endPoint(), shapes[j2].second->endPoint(), shapes[j2].second->startPoint(), &tab, &tcd, false, int_pt)) {
					bool valid = false;
					int start_index = shapes[j].second->startIndex();
					int end_index = shapes[j2].second->endIndex();
					if (start_index > end_index) end_index += polygon.contour.size();
					for (int k = start_index; k <= end_index; k++) {
						int index = (k + polygon.contour.size()) % polygon.contour.size();
						if (cv::norm(polygon.contour[index] - int_pt) < max_error) {
							valid = true;
							break;
						}
					}

					if (valid) {
						contour.push_back(int_pt);
						continue;
					}
				}
				*/
				// simpliy connect two shapes
				contour.push_back(line->endPoint());
				contour.push_back(line2->startPoint());
			}
		}
	}

}