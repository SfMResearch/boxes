
#ifndef BOXES_CONVERTERS_H
#define BOXES_CONVERTERS_H

#ifdef BOXES_PRIVATE

#include <opencv2/opencv.hpp>
#include <vector>

#endif

namespace Boxes {

#ifdef BOXES_PRIVATE

// Convert degree to rad.
#ifndef DEG2RAD
#define DEG2RAD(x) (M_PI / (180 * x))
#endif

#define BOXES_EPSILON 10e-9

#define IS_ZERO(x) (fabsf(x) <= BOXES_EPSILON)
#define GREATER_OR_EQUAL_TO_ZERO(x) (x >= -BOXES_EPSILON)

	// KeyPoints to Points
	inline std::vector<cv::Point2f> convertKeyPoints(const std::vector<cv::KeyPoint>* keypoints) {
		std::vector<cv::Point2f> points;

		for (std::vector<cv::KeyPoint>::const_iterator i = keypoints->begin(); i != keypoints->end(); ++i) {
			points.push_back(i->pt);
		}

		return points;
	}

	inline cv::Matx34d merge_rotation_and_translation_matrix(const cv::Mat_<double>* rotation, const cv::Mat_<double>* translation) {
		cv::Matx34d matrix = cv::Matx34d(
			rotation->at<double>(0, 0), rotation->at<double>(0, 1), rotation->at<double>(0, 2), translation->at<double>(0),
			rotation->at<double>(1, 0), rotation->at<double>(1, 1), rotation->at<double>(1, 2), translation->at<double>(1),
			rotation->at<double>(2, 0), rotation->at<double>(2, 1), rotation->at<double>(2, 2), translation->at<double>(2)
		);

		return matrix;
	}
#endif

};

#endif
