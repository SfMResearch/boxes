
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
			double reprojection_error;

			// Methods
			bool rotation_is_coherent() const;
			double percentage_of_points_in_front_of_camera() const;
	};
};

#endif
