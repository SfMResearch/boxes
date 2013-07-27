
#ifndef BOXES_STRUCTS_H
#define BOXES_STRUCTS_H

#include <opencv2/opencv.hpp>
#include <vector>

namespace Boxes {
	struct CloudPoint {
		cv::Point3d pt;
		// std::vector<int> imgpt_for_img;
		double reprojection_error;
	};

	struct MatchPoint {
		cv::Point2f pt;
		int query_index = -1;
	};
};

#endif
