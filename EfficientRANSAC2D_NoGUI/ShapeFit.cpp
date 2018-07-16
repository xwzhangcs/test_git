#include "ShapeFit.h"

ShapeFit::ShapeFit() {
}

ShapeFit::~ShapeFit() {
}

std::vector<cv::Point2f> ShapeFit::fit(const std::vector<cv::Point2f>& polygon, const std::vector<cv::Point2f>& ini_points) {
	float min_x = std::numeric_limits<float>::max();
	float min_y = std::numeric_limits<float>::max();
	float max_x = -std::numeric_limits<float>::max();
	float max_y = -std::numeric_limits<float>::max();
	std::cout << "size is " << polygon.size() << std::endl;
	for (int i = 0; i < polygon.size(); i++) {
		min_x = std::min(min_x, polygon[i].x);
		min_y = std::min(min_y, polygon[i].y);
		max_x = std::max(max_x, polygon[i].x);
		max_y = std::max(max_y, polygon[i].y);
	}
	for (int i = 0; i < ini_points.size(); i++) {
		min_x = std::min(min_x, ini_points[i].x);
		min_y = std::min(min_y, ini_points[i].y);
		max_x = std::max(max_x, ini_points[i].x);
		max_y = std::max(max_y, ini_points[i].y);
	}
	std::vector<cv::Point2f> normalized_polygon(polygon.size());
	for (int i = 0; i < polygon.size(); i++) {
		normalized_polygon[i] = cv::Point2f((polygon[i].x - min_x) / (max_x - min_x), (polygon[i].y - min_y) / (max_y - min_y));
	}


	std::vector<cv::Point2f> normalized_polygon_tmp(ini_points.size());
	for (int i = 0; i < ini_points.size(); i++) {
		normalized_polygon_tmp[i] = cv::Point2f((ini_points[i].x - min_x) / (max_x - min_x), (ini_points[i].y - min_y) / (max_y - min_y));
	}

	try {
		/*
		column_vector starting_point(8);
		starting_point(0) = 0.2;
		starting_point(1) = 0.2;
		starting_point(2) = 0.8;
		starting_point(3) = 0.2;
		starting_point(4) = 0.8;
		starting_point(5) = 0.8;
		starting_point(6) = 0.2;
		starting_point(7) = 0.8;
		*/

		/*column_vector starting_point(16);
		starting_point(0) = 0.3;
		starting_point(1) = 0.1;
		starting_point(2) = 0.7;
		starting_point(3) = 0.1;
		starting_point(4) = 0.9;
		starting_point(5) = 0.3;
		starting_point(6) = 0.9;
		starting_point(7) = 0.7;
		starting_point(8) = 0.7;
		starting_point(9) = 0.9;
		starting_point(10) = 0.3;
		starting_point(11) = 0.9;
		starting_point(12) = 0.1;
		starting_point(13) = 0.7;
		starting_point(14) = 0.1;
		starting_point(15) = 0.3;*/
		column_vector starting_point(normalized_polygon_tmp.size() * 2);
		for (int i = 0; i < normalized_polygon_tmp.size(); i++){
			starting_point(i * 2) = normalized_polygon_tmp[i].x;
			starting_point(i * 2 + 1) = normalized_polygon_tmp[i].y;
		}

		BFGSSolver solver(normalized_polygon);
		find_max_using_approximate_derivatives(dlib::bfgs_search_strategy(), dlib::objective_delta_stop_strategy(1e-7),	solver, starting_point, 1, 0.0001);

		std::vector<cv::Point2f> ans(starting_point.size() / 2);
		for (int i = 0; i < starting_point.size() / 2; i++) {
			ans[i].x = starting_point(i * 2) * (max_x - min_x) + min_x;
			ans[i].y = starting_point(i * 2 + 1) * (max_y - min_y) + min_y;
		}

		return ans;
	}
	catch (char* ex) {
		std::cout << ex << std::endl;
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}
	catch (...) {
		std::cout << "BFGS optimization failure." << std::endl;
	}

	return{};
}