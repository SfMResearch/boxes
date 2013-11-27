
#include <opencv2/opencv.hpp>
#include <vector>

#include <boxes/boxes.h>
#include <boxes/camera_matrix.h>
#include <boxes/cloud_point.h>

namespace Boxes {
	/*
	 * Contructor.
	 */
	CameraMatrix::CameraMatrix(Boxes* boxes) {
		cv::Matx34d matrix = cv::Matx34d(
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0
		);

		this->init(boxes, matrix);
	}

	CameraMatrix::CameraMatrix(Boxes* boxes, CameraMatrix* old) {
		this->init(boxes, old->matrix);
	}

	CameraMatrix::CameraMatrix(Boxes* boxes, cv::Matx34d matrix) {
		this->init(boxes, matrix);
	}

	CameraMatrix::~CameraMatrix() {
		delete this->point_cloud;
	}

	void CameraMatrix::init(Boxes* boxes, cv::Matx34d matrix) {
		this->boxes = boxes;

		this->matrix = matrix;

		this->point_cloud = new PointCloud(this->boxes);
	}

	cv::Mat CameraMatrix::get_rotation_matrix() const {
		return cv::Mat(this->matrix).colRange(0, 3);
	}

	cv::Mat CameraMatrix::get_translation_vector() const {
		return cv::Mat(this->matrix).col(3);
	}

	double CameraMatrix::get_rotation_determinant() const {
		cv::Mat rotation = this->get_rotation_matrix();

		return cv::determinant(rotation);
	}

	bool CameraMatrix::rotation_is_coherent() const {
		double determinant = this->get_rotation_determinant();

		// Must be +1 or -1
		determinant = fabsf(determinant) - 1.0;

		if (IS_ZERO(determinant)) {
			return true;
		}

		return false;
	}

	double CameraMatrix::percentage_of_points_in_front_of_camera() const {
		/* Check if there are actually points in the cloud.
		 * If not, we return right here, because cv::perspectiveTransform
		 * will raise an exception when called with an empty array of points.
		 */
		if (this->point_cloud->size() == 0) {
			return 0.0;
		}

		unsigned int points_in_front = 0;

		cv::Matx44d matrix_mul = cv::Matx44d::eye();
		for (unsigned int i = 0; i < 12; i++) {
			matrix_mul(i) = this->matrix(i);
		}

		std::vector<cv::Point3d> points;
		std::vector<cv::Point3d> points_transformed;

		const std::vector<CloudPoint>* p = this->point_cloud->get_points();
		for (std::vector<CloudPoint>::const_iterator cp = p->begin(); cp != p->end(); ++cp) {
			points.push_back(cp->pt);
		}

		cv::perspectiveTransform(points, points_transformed, matrix_mul);

		for (unsigned int i = 0; i <points.size();i++) {
			if (points[i].z > 0 && points_transformed[i].z > 0)
				points_in_front++;
		}

		return (double)points_in_front / ((double)points.size());
	}
}
