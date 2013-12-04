
#ifndef BOXES_STRUCTS_H
#define BOXES_STRUCTS_H

#include <opencv2/opencv.hpp>

namespace Boxes {
	struct MatchPoint {
		cv::Point2f pt1;
		cv::Point2f pt2;

		double distance = 0.0;
	};
};

#endif
