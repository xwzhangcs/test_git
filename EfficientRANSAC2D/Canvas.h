#ifndef CANVAS_H
#define CANVAS_H

#include <vector>
#include <QWidget>
#include <QKeyEvent>
#include "../EfficientRANSAC2D_NoGUI/CurveDetector.h"
#include "../EfficientRANSAC2D_NoGUI/LineDetector.h"
#include "../EfficientRANSAC2D_NoGUI/OrientationEstimator.h"
#include "../EfficientRANSAC2D_NoGUI/PrimitiveShape.h"
#include "../EfficientRANSAC2D_NoGUI/ContourGenerator.h"
#include "../EfficientRANSAC2D_NoGUI/EfficientRANSAC.h"

class Canvas : public QWidget {
private:
	QImage orig_image;
	QImage image;
	float image_scale;
	std::vector<util::Polygon> polygons;
	std::vector<std::vector<std::pair<int, std::shared_ptr<efficient_ransac::PrimitiveShape>>>> shapes;
	std::vector<std::vector<cv::Point2f>> contours;

	bool ctrlPressed;
	bool shiftPressed;
	
public:
	Canvas(QWidget *parent = NULL);

	void loadImage(const QString& filename);
	void saveImage(const QString& filename);
	void detectContours();
	void detectCurves(int num_iterations, int min_points, float max_error_ratio_to_radius, float cluster_epsilon, float min_angle, float min_radius, float max_radius);
	void detectLines(int num_iterations, int min_points, float max_error, float cluster_epsilon, float min_length, float angle_threshold);
	void rightAngle(int resolution, bool optimization);
	void detectCurvesLines(int curve_num_iterations, int curve_min_points, float curve_max_error_ratio_to_radius, float curve_cluster_epsilon, float curve_min_angle, float curve_min_radius, float curve_max_radius, int line_num_iterations, int line_min_points, float line_max_error, float line_cluster_epsilon, float line_min_length, float line_angle_threshold);
	void generateContours(int curve_num_iterations, int curve_min_points, float curve_max_error_ratio_to_radius, float curve_cluster_epsilon, float curve_min_angle, float curve_min_radius, float curve_max_radius, int line_num_iterations, int line_min_points, float line_max_error, float line_cluster_epsilon, float line_min_length, float line_angle_threshold, float contour_max_error, float contour_angle_threshold);
	void keyPressEvent(QKeyEvent* e);
	void keyReleaseEvent(QKeyEvent* e);

protected:
	void paintEvent(QPaintEvent *event);
	void mousePressEvent(QMouseEvent* e);
	void resizeEvent(QResizeEvent *e);
};

#endif // CANVAS_H
