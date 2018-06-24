#pragma once

#include <vector>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

class Point {
public:
	cv::Point2f pos;
	cv::Point2f normal;
	bool used;

public:
	Point() : used(false) {}
	Point(float x, float y) : pos(x, y), used(false) {}
	Point(float x, float y, float nx, float ny) : pos(x, y), normal(nx, ny), used(false) {}
};

class Polygon {
public:
	std::vector<Point> contour;
	std::vector<std::vector<Point>> holes;

public:
	Polygon() {}

	void clearUsedFlag() {
		for (auto& pt : contour) pt.used = false;
		for (auto& hole : holes) {
			for (auto& pt : hole) pt.used = false;
		}
	}
};

std::vector<Polygon> findContours(const cv::Mat& image, int threshold, bool simplify);
float angle_difference(float x, float xi);
float regularize_angle_PI(float x);
bool lineLineIntersection(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c, const cv::Point2f& d, double *tab, double *tcd, bool segment_only, cv::Point2f& int_pt);
