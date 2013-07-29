
#include <opencv2/opencv.hpp>
#include <vector>

#include <boxes/camera_matrix.h>
#include <boxes/structs.h>

namespace Boxes {
	/*
	 * Contructor.
	 */
	CameraMatrix::CameraMatrix(cv::Matx34d matrix) {
		this->matrix = matrix;
	}
}
