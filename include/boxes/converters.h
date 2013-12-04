
#ifndef BOXES_CONVERTERS_H
#define BOXES_CONVERTERS_H

#ifdef BOXES_PRIVATE

#include <opencv2/opencv.hpp>
#include <vector>

#include <boxes/structs.h>

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

	inline cv::Mat normalize(cv::Mat input) {
		cv::Mat output;
		cv::normalize(input, output);

		return output;
	}

	inline int search_matching_keypoint_index(cv::Point2f pt, const std::vector<cv::KeyPoint>* keypoints) {
		int counter = 0;

		for (std::vector<cv::KeyPoint>::const_iterator i = keypoints->begin(); i != keypoints->end(); i++) {
			if (i->pt == pt)
				return counter;

			counter++;
		}

		return -1;
	}

	inline cv::DMatch match_point_to_dmatch(MatchPoint* mp, const std::vector<cv::KeyPoint>* keypoints1, const std::vector<cv::KeyPoint>* keypoints2) {
		cv::DMatch match;

		int keypoint1_idx = search_matching_keypoint_index(mp->pt1, keypoints1);
		int keypoint2_idx = search_matching_keypoint_index(mp->pt2, keypoints2);

		assert(keypoint1_idx >= 0 && keypoint2_idx >= 0);

		match.queryIdx = keypoint1_idx;
		match.trainIdx = keypoint2_idx;
		match.distance = mp->distance;

		return match;
	}
#endif

};

#endif
