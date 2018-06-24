#include "Util.h"

/**
 * Find the contour polygons from the image.
 */
std::vector<Polygon> findContours(const cv::Mat& image, int threshold, bool simplify) {
	std::vector<Polygon> polygons;

	cv::Mat mat = image.clone();
	cv::threshold(mat, mat, threshold, 255, cv::THRESH_BINARY);

	// extract contours
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	if (simplify) {
		cv::findContours(mat, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	}
	else {
		cv::findContours(mat, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
	}

	for (int i = 0; i < hierarchy.size(); i++) {
		if (hierarchy[i][3] != -1) continue;
		if (contours[i].size() < 3) continue;

		Polygon polygon;
		polygon.contour.resize(contours[i].size());
		for (int j = 0; j < contours[i].size(); j++) {
			polygon.contour[j] = Point(contours[i][j].x, contours[i][j].y);
		}

		if (polygon.contour.size() >= 3) {
			// obtain all the holes inside this contour
			int hole_id = hierarchy[i][2];
			while (hole_id != -1) {
				std::vector<Point> hole;
				hole.resize(contours[hole_id].size());
				for (int j = 0; j < contours[hole_id].size(); j++) {
					hole[j] = Point(contours[hole_id][j].x, contours[hole_id][j].y);
				}
				polygon.holes.push_back(hole);
				hole_id = hierarchy[hole_id][0];
			}

			polygons.push_back(polygon);
		}
	}

	return polygons;
}

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