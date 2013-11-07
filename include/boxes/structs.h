
#ifndef BOXES_STRUCTS_H
#define BOXES_STRUCTS_H

#include <opencv2/opencv.hpp>

namespace Boxes {
	struct MatchPoint {
		cv::Point2f pt;
		int query_index = -1;
	};
};

#endif
