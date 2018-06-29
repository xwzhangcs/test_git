#pragma once

#include <vector>
#include <memory>
#include "EfficientRANSAC.h"

namespace efficient_ransac {

	class Line : public PrimitiveShape {
	private:
		/** Any point on the line */
		cv::Point2f _point;

		/** Orientation vector of the line */
		cv::Point2f _dir;

		/** Length of the line segment */
		float _length;

	public:
		Line();
		Line(int index, const cv::Point2f& point, const cv::Point2f& dir);
		~Line() {}

		cv::Point2f point() { return _point; }
		cv::Point2f dir() { return _dir; }
		void setDir(const cv::Point2f& dir) { _dir = dir; }
		float length() { return _length; }

		float distance(const cv::Point2f& p) {
			return std::abs((p - _point).dot(cv::Point2f(_dir.y, -_dir.x)));
		}

		void setEndPositions(const std::vector<cv::Point2f>& points) {
			float _start_pos = std::numeric_limits<float>::max();
			float _end_pos = -std::numeric_limits<float>::max();

			for (auto& pt : points) {
				float t = (pt - _point).dot(_dir);
				_start_pos = std::min(_start_pos, t);
				_end_pos = std::max(_end_pos, t);
			}
			if (points.size() > 0) {
				start_point = _point + _dir * (points[0] - _point).dot(_dir);
				end_point = _point + _dir * (points.back() - _point).dot(_dir);
			}
			_length = _end_pos - _start_pos;
		}
	};

	class LineDetector {
	protected:
		LineDetector() {}

	public:
		static void detect(std::vector<Point>& polygon, int num_iter, int min_points, float max_error, float cluster_epsilon, float min_length, std::vector<float>& principal_angles, float angle_threshold, std::vector<std::pair<int, std::shared_ptr<PrimitiveShape>>>& lines);
	};

}