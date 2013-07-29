
#ifndef BOXES_CAMERA_MATRIX_H
#define BOXES_CAMERA_MATRIX_H

#include <opencv2/opencv.hpp>
#include <vector>

#include <boxes/constants.h>
#include <boxes/image.h>
#include <boxes/structs.h>

namespace Boxes {
	class CameraMatrix {
		public:
			CameraMatrix(cv::Matx34d matrix);

			cv::Matx34d matrix;
			std::vector<CloudPoint> point_cloud;
			std::vector<cv::KeyPoint> corresponding_image_points;

			// Methods
			bool rotation_is_coherent() const;
	};
};

#endif
