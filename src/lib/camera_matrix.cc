
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

	bool CameraMatrix::rotation_is_coherent() const {
		cv::Mat rotation = cv::Mat(this->matrix).colRange(0, 3);

		double determinant = fabsf(cv::determinant(rotation)) - 1.0;
		return (determinant > 1e-07);
	}
}
