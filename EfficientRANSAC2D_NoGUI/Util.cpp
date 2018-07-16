#include "Util.h"

namespace util {

	Polygon::Polygon() {
	}

	/**
	 * Find the contour polygons from the image.
	 */
	std::vector<Polygon> findContours(const cv::Mat_<uchar>& image, int threshold, bool simplify, bool allow_diagonal, bool dilate) {
		std::vector<Polygon> polygons;

		cv::Mat mat = image.clone();
		cv::threshold(mat, mat, threshold, 255, cv::THRESH_BINARY);

		cv::Mat_<uchar> mat2 = mat.clone();

		// if diagonal is not allowwd, dilate the diagonal connection.
		if (!allow_diagonal) {
			while (true) {
				bool updated = false;
				for (int r = 0; r < mat.rows - 1; r++) {
					for (int c = 0; c < mat.cols - 1; c++) {
						if (mat2(r, c) == 255 && mat2(r + 1, c + 1) == 255 && mat2(r + 1, c) == 0 && mat2(r, c + 1) == 0) {
							updated = true;
							mat2(r + 1, c) = 255;
						}
						else if (mat2(r, c) == 0 && mat2(r + 1, c + 1) == 0 && mat2(r + 1, c) == 255 && mat2(r, c + 1) == 255) {
							updated = true;
							mat2(r, c) = 255;
						}
					}
				}
				if (!updated) break;
			}
		}

		// dilate the image
		if (dilate) {
			// resize x4
			cv::Mat_<uchar> img3;
			cv::resize(mat2, img3, cv::Size(mat2.cols * 4, mat2.rows * 4), 0, 0, cv::INTER_NEAREST);

			// add padding
			cv::Mat_<uchar> padded = cv::Mat_<uchar>::zeros(img3.rows + 1, img3.cols + 1);
			img3.copyTo(padded(cv::Rect(0, 0, img3.cols, img3.rows)));

			// dilate image
			cv::Mat_<uchar> kernel = (cv::Mat_<uchar>(3, 3) << 1, 1, 0, 1, 1, 0, 0, 0, 0);
			cv::dilate(padded, mat2, kernel);
		}

		// extract contours
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;
		if (simplify) {
			cv::findContours(mat2, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
		}
		else {
			cv::findContours(mat2, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
		}

		for (int i = 0; i < hierarchy.size(); i++) {
			if (hierarchy[i][3] != -1) continue;
			if (contours[i].size() < 3) continue;

			util::Polygon polygon;
			polygon.contour.resize(contours[i].size());
			for (int j = 0; j < contours[i].size(); j++) {
				if (dilate) {
					polygon.contour[j] = cv::Point2f(std::round(contours[i][j].x * 0.25), std::round(contours[i][j].y * 0.25));
				}
				else {
					polygon.contour[j] = cv::Point2f(contours[i][j].x, contours[i][j].y);
				}
			}
			if (dilate) {
				polygon.contour = removeRedundantPoint(polygon.contour);
			}

			if (polygon.contour.size() >= 3) {
				// obtain all the holes inside this contour
				int hole_id = hierarchy[i][2];
				while (hole_id != -1) {
					// ignore holes in this implementation for simplicity

					hole_id = hierarchy[hole_id][0];
				}

				polygons.push_back(polygon);
			}
		}

		return polygons;

	}

	/**
	* Remove the redundant points that have the same coordinates, and remove the point that are collinear to the adjacent points.
	*/
	std::vector<cv::Point2f> removeRedundantPoint(const std::vector<cv::Point2f>& polygon) {
		std::vector<cv::Point2f> ans;
		if (polygon.size() == 0) return ans;

		ans.push_back(polygon[0]);
		for (int i = 1; i < polygon.size(); i++) {
			if (polygon[i] != polygon[i - 1]) {
				ans.push_back(polygon[i]);
			}
		}
		if (ans.size() > 1 && ans.back() == ans.front()) ans.pop_back();

		for (int i = 0; i < ans.size() && ans.size() >= 3;) {
			int prev = (i - 1 + ans.size()) % ans.size();
			int next = (i + 1) % ans.size();
			if ((ans[i] - ans[prev]).dot(ans[next] - ans[i]) > 0 && std::abs((ans[i] - ans[prev]).cross(ans[next] - ans[i])) < 0.0001) {
				ans.erase(ans.begin() + i);
			}
			else {
				i++;
			}
		}

		return ans;
	}

	std::vector<cv::Point2f> simplifyContour(const std::vector<cv::Point2f>& polygon, float epsilon) {
		std::vector<cv::Point2f> ans;
		cv::approxPolyDP(polygon, ans, epsilon, true);
		if (isSimple(ans)) return ans;
		else return polygon;
	}

	cv::Rect boundingBox(const std::vector<cv::Point2f>& polygon) {
		int min_x = std::numeric_limits<int>::max();
		int max_x = -std::numeric_limits<int>::max();
		int min_y = std::numeric_limits<int>::max();
		int max_y = -std::numeric_limits<int>::max();
		for (int i = 0; i < polygon.size(); i++) {
			min_x = std::min(min_x, (int)std::floor(polygon[i].x));
			max_x = std::max(max_x, (int)std::ceil(polygon[i].x));
			min_y = std::min(min_y, (int)std::floor(polygon[i].y));
			max_y = std::max(max_y, (int)std::ceil(polygon[i].y));
		}

		return cv::Rect(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1);
	}

	cv::Rect boundingBox(const std::vector<cv::Point>& polygon) {
		int min_x = std::numeric_limits<int>::max();
		int max_x = -std::numeric_limits<int>::max();
		int min_y = std::numeric_limits<int>::max();
		int max_y = -std::numeric_limits<int>::max();
		for (int i = 0; i < polygon.size(); i++) {
			min_x = std::min(min_x, (int)std::floor(polygon[i].x));
			max_x = std::max(max_x, (int)std::ceil(polygon[i].x));
			min_y = std::min(min_y, (int)std::floor(polygon[i].y));
			max_y = std::max(max_y, (int)std::ceil(polygon[i].y));
		}

		return cv::Rect(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1);
	}

	/**
	* Create image from the contour.
	*/
	void createImageFromContour(int width, int height, const std::vector<cv::Point>& contour, const cv::Point& offset, cv::Mat_<uchar>& result, bool erode) {
		result = cv::Mat_<uchar>::zeros(height * 2, width * 2);
		std::vector<std::vector<cv::Point>> contour_points(1);

		contour_points[0].resize(contour.size());
		for (int i = 0; i < contour.size(); i++) {
			contour_points[0][i] = cv::Point((contour[i].x + offset.x) * 2, (contour[i].y + offset.y) * 2);
		}

		cv::fillPoly(result, contour_points, cv::Scalar(255), cv::LINE_4);

		// erode image
		if (erode) {
			cv::Mat_<uchar> kernel = (cv::Mat_<uchar>(3, 3) << 0, 0, 0, 0, 0, 1, 0, 1, 1);
			cv::erode(result, result, kernel);
		}

		cv::resize(result, result, cv::Size(width, height), 0, 0, cv::INTER_NEAREST);
	}

	/**
	* Calculate the intersection over union (IOU) inside the specified roi.
	* The images have to be CV_8U type.
	*/
	float calculateIOU(const cv::Mat_<uchar>& img, const cv::Mat_<uchar>& img2) {
		float union_cnt = 0;
		float inter_cnt = 0;
		for (int r = 0; r < img.rows; r++) {
			for (int c = 0; c < img.cols; c++) {
				union_cnt += std::max(img.at<uchar>(r, c), img2.at<uchar>(r, c));
				inter_cnt += std::min(img.at<uchar>(r, c), img2.at<uchar>(r, c));
			}
		}

		return (float)inter_cnt / union_cnt;
	}

	float calculateIOU(const std::vector<cv::Point2f>& polygon1, const std::vector<cv::Point2f>& polygon2) {
		if (polygon1.size() == 0 || polygon2.size() == 0) return 0;

		CGAL::Polygon_2<Kernel> pol1;
		for (auto& pt : polygon1) {
			pol1.push_back(Kernel::Point_2(pt.x, pt.y));
		}
		if (!pol1.is_simple()) {
			throw "calculateIOU: polygon1 is not simple.";
		}
		if (pol1.is_clockwise_oriented()) pol1.reverse_orientation();

		CGAL::Polygon_2<Kernel> pol2;
		for (auto& pt : polygon2) {
			pol2.push_back(Kernel::Point_2(pt.x, pt.y));
		}
		if (!pol2.is_simple()) {
			throw "calculateIOU: polygon2 is not simple.";
		}
		if (pol2.is_clockwise_oriented()) pol2.reverse_orientation();

		if (CGAL::do_intersect(pol1, pol2)) {
			std::list<CGAL::Polygon_with_holes_2<Kernel>> inter;
			CGAL::intersection(pol1, pol2, std::back_inserter(inter));

			CGAL::Polygon_with_holes_2<Kernel> uni;
			CGAL::join(pol1, pol2, uni);

			float inter_area = 0;
			for (auto it = inter.begin(); it != inter.end(); it++) {
				inter_area += area(*it);
			}

			return inter_area / area(uni);
		}
		else {
			return 0;
		}
	}

	float calculateIOUbyImage(const std::vector<cv::Point2f>& polygon1, const std::vector<cv::Point2f>& polygon2) {
		int min_x = INT_MAX;
		int min_y = INT_MAX;
		int max_x = INT_MIN;
		int max_y = INT_MIN;
		for (int i = 0; i < polygon1.size(); i++) {
			min_x = std::min(min_x, (int)polygon1[i].x);
			min_y = std::min(min_y, (int)polygon1[i].y);
			max_x = std::max(max_x, (int)(polygon1[i].x + 0.5));
			max_y = std::max(max_y, (int)(polygon1[i].y + 0.5));
		}
		for (int i = 0; i < polygon2.size(); i++) {
			min_x = std::min(min_x, (int)polygon2[i].x);
			min_y = std::min(min_y, (int)polygon2[i].y);
			max_x = std::max(max_x, (int)(polygon2[i].x + 0.5));
			max_y = std::max(max_y, (int)(polygon2[i].y + 0.5));
		}

		cv::Mat_<uchar> img1 = cv::Mat_<uchar>::zeros(max_y - min_y + 1, max_x - min_x + 1);
		if (polygon1.size() > 0) {
			std::vector<std::vector<cv::Point>> contour_points1(1);
			contour_points1[0].resize(polygon1.size());
			for (int i = 0; i < polygon1.size(); i++) {
				contour_points1[0][i] = cv::Point(polygon1[i].x - min_x, polygon1[i].y - min_y);
			}
			cv::fillPoly(img1, contour_points1, cv::Scalar(255), cv::LINE_4);
		}

		cv::Mat_<uchar> img2 = cv::Mat_<uchar>::zeros(max_y - min_y + 1, max_x - min_x + 1);
		if (polygon2.size() > 0) {
			std::vector<std::vector<cv::Point>> contour_points2(1);
			contour_points2[0].resize(polygon2.size());
			for (int i = 0; i < polygon2.size(); i++) {
				contour_points2[0][i] = cv::Point(polygon2[i].x - min_x, polygon2[i].y - min_y);
			}
			cv::fillPoly(img2, contour_points2, cv::Scalar(255), cv::LINE_4);
		}

		int inter_cnt = 0;
		int union_cnt = 0;
		for (int r = 0; r < img1.rows; r++) {
			for (int c = 0; c < img1.cols; c++) {
				if (img1(r, c) == 255 && img2(r, c) == 255) inter_cnt++;
				if (img1(r, c) == 255 || img2(r, c) == 255) union_cnt++;
			}
		}

		return (float)inter_cnt / union_cnt;
	}

	float area(const CGAL::Polygon_with_holes_2<Kernel>& pwh) {
		float ans = 0;

		ans += CGAL::to_double(pwh.outer_boundary().area());
		for (auto it = pwh.holes_begin(); it != pwh.holes_end(); it++) {
			ans -= CGAL::to_double(it->area());
		}

		return ans;
	}

	float area(const std::vector<cv::Point2f>& polygon) {
		CGAL::Polygon_2<Kernel> pgn;
		for (int i = 0; i < polygon.size(); i++) {
			pgn.push_back(Kernel::Point_2(polygon[i].x, polygon[i].y));
		}
		return std::abs(CGAL::to_double(pgn.area()));
	}

	bool isSimple(const std::vector<cv::Point2f>& polygon) {
		CGAL::Polygon_2<Kernel> pgn;
		for (int i = 0; i < polygon.size(); i++) {
			pgn.push_back(Kernel::Point_2(polygon[i].x, polygon[i].y));
		}
		return pgn.is_simple();
	}

	bool isSimple(const std::vector<cv::Point>& polygon) {
		CGAL::Polygon_2<Kernel> pgn;
		for (int i = 0; i < polygon.size(); i++) {
			pgn.push_back(Kernel::Point_2(polygon[i].x, polygon[i].y));
		}
		return pgn.is_simple();
	}

	void counterClockwise(std::vector<cv::Point2f>& polygon) {
		CGAL::Polygon_2<Kernel> pgn;
		for (int i = 0; i < polygon.size(); i++) {
			pgn.push_back(Kernel::Point_2(polygon[i].x, polygon[i].y));
		}
		if (pgn.is_clockwise_oriented()) {
			std::reverse(polygon.begin(), polygon.end());
		}
	}

	void transform(Polygon& polygon, const cv::Mat_<float>& M) {
		transform(polygon.contour, M);
		for (auto& hole : polygon.holes) {
			transform(hole, M);
		}
	}

	void transform(std::vector<cv::Point2f>& polygon, const cv::Mat_<float>& M) {
		for (auto& pt : polygon) {
			cv::Mat_<float> p = (cv::Mat_<float>(3, 1) << pt.x, pt.y, 1);
			cv::Mat_<float> p2 = M * p;
			pt = cv::Point2f(p2(0, 0), p2(1, 0));
		}
	}

	bool withinPolygon(const cv::Point2f& pt, const Polygon& polygon) {
		if (!withinPolygon(pt, polygon.contour)) return false;

		for (int i = 0; i < polygon.holes.size(); i++) {
			if (withinPolygon(pt, polygon.holes[i])) return false;
		}

		return true;
	}

	bool withinPolygon(const cv::Point2f& pt, const std::vector<cv::Point2f>& polygon) {
		CGAL::Polygon_2<Kernel> pgon;
		for (int i = 0; i < polygon.size(); i++) {
			pgon.push_back(Kernel::Point_2(polygon[i].x, polygon[i].y));
		}

		return pgon.bounded_side(Kernel::Point_2(pt.x, pt.y)) != CGAL::ON_UNBOUNDED_SIDE;
	}

	bool withinPolygon(const std::vector<cv::Point2f>& inner_polygon, const std::vector<cv::Point2f>& outer_polygon) {
		for (int i = 0; i < inner_polygon.size(); i++) {
			if (!withinPolygon(inner_polygon[i], outer_polygon)) return false;
		}
		return true;
	}

	/**
	 * Calculate the intersection of line a-b and line c-d.
	 *
	 * @param a				the first end point of the first line
	 * @param b				the second end point of the first line
	 * @param c				the first end point of the second line
	 * @param d				the second end point of the second line
	 * @param tab			the position of the intersecion on the first line (0 means point a, whereas 1 means point b).
	 * @param tcd			the position of the intersecion on the second line (0 means point c, whereas 1 means point d).
	 * @param segment_only	if true, find the intersection only on the line segments.
	 * @param int_pt		intersecion point
	 */
	bool lineLineIntersection(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c, const cv::Point2f& d, double *tab, double *tcd, bool segment_only, cv::Point2f& int_pt) {
		cv::Point2f u = b - a;
		cv::Point2f v = d - c;

		if (cv::norm(u) < 0.0000001 || cv::norm(v) < 0.0000001) {
			return false;
		}

		double numer = v.x * (c.y - a.y) + v.y * (a.x - c.x);
		double denom = u.y * v.x - u.x * v.y;

		if (denom == 0.0)  {
			// they are parallel
			return false;
		}

		double t0 = numer / denom;

		cv::Point2f ipt = a + t0*u;
		cv::Point2f tmp = ipt - c;
		double t1;
		if (tmp.dot(v) > 0.0) {
			t1 = cv::norm(tmp) / cv::norm(v);
		}
		else {
			t1 = -1.0 * cv::norm(tmp) / cv::norm(d - c);
		}

		// check if intersection is within the segments
		if (segment_only && !((t0 >= 0.0000001) && (t0 <= 1.0 - 0.0000001) && (t1 >= 0.0000001) && (t1 <= 1.0 - 0.0000001))) {
			return false;
		}

		*tab = t0;
		*tcd = t1;
		cv::Point2f dirVec = b - a;

		int_pt = a + t0 * dirVec;

		return true;
	}

}