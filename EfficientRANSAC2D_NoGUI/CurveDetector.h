#pragma once

#include <vector>
#include <memory>
#include "EfficientRANSAC.h"

namespace efficient_ransac {

	class Circle : public PrimitiveShape {
	private:
		/** Center of the circle */
		cv::Point2f _center;

		/** Radius of the circle */
		float _radius;

		/** Start angle that corresponds to start_point */
		float _start_angle;

		/** End angle that corresponds to end_point */
		float _end_angle;

		/** Angle range */
		float _angle_range;

		/** Signed angle range */
		float _signed_angle_range;

	public:
		Circle();
		Circle(int index, const cv::Point2f& point, const cv::Point2f& center, float radius);
		~Circle() {}

		cv::Point2f center() { return _center; }
		float radius() { return _radius; }
		float startAngle() { return _start_angle; }
		float endAngle() { return _end_angle; }
		float angleRange() { return _angle_range; }
		float signedAngleRange() { return _signed_angle_range; }

		float distance(const cv::Point2f& p) {
			return std::abs(cv::norm(p - _center) - _radius);
		}

		void setMinMaxAngles(std::vector<float>& angles) {
			if (angles.size() == 0) return;
			else if (angles.size() == 1) {
				_start_angle = angles[0];
				_end_angle = angles[0];
				_angle_range = 0;
				_signed_angle_range = 0;
				return;
			}

			std::sort(angles.begin(), angles.end());

			_start_angle = angles[0];
			_end_angle = angles.back();
			float max_gap = angles[0] + CV_PI * 2 - angles.back();
			_angle_range = _end_angle - _start_angle;

			std::sort(angles.begin(), angles.end());
			for (int i = 1; i < angles.size(); i++) {
				if (angles[i] - angles[i - 1] > max_gap) {
					max_gap = angles[i] - angles[i - 1];
					_start_angle = angles[i];
					_end_angle = angles[i - 1] + CV_PI * 2;
					_angle_range = _end_angle - _start_angle;
				}
			}

			// if the arc is almost a complete circle, make it a circle
			if (_angle_range > CV_PI * 1.9) {
				_end_angle = _start_angle + CV_PI * 2;
				_angle_range = CV_PI * 2;
			}

			// swap start_angle and end_angle accroding to the start_point and end_point
			cv::Point2f p1 = _center + _radius * cv::Point2f(std::cos(_start_angle), std::sin(_start_angle));
			cv::Point2f p2 = _center + _radius * cv::Point2f(std::cos(_end_angle), std::sin(_end_angle));
			if (cv::norm(start_point - p1) > cv::norm(start_point - p2)) {
				std::swap(_start_angle, _end_angle);
			}

			if (_start_angle < _end_angle) {
				_signed_angle_range = _angle_range;
			}
			else {
				_signed_angle_range = -_angle_range;
			}
		}
	};

	class CurveDetector {
	protected:
		CurveDetector() {}

	public:
		static void detect(std::vector<Point>& polygon, int num_iter, int min_points, float max_error_ratio_to_radius, float cluster_epsilon, float min_angle, float min_radius, float max_radius, std::vector<std::pair<int, std::shared_ptr<PrimitiveShape>>>& circles);
		static std::shared_ptr<Circle> circleFromPoints(int index, const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& p3);
		static float crossProduct(const cv::Point2f& a, const cv::Point2f& b);
	};

}