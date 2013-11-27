
#ifndef BOXES_CAMERA_MATRIX_H
#define BOXES_CAMERA_MATRIX_H

#include <opencv2/opencv.hpp>
#include <vector>

#include <boxes/forward_declarations.h>
#include <boxes/boxes.h>
#include <boxes/constants.h>
#include <boxes/image.h>
#include <boxes/point_cloud.h>
#include <boxes/structs.h>

namespace Boxes {
	class CameraMatrix {
		public:
			CameraMatrix(Boxes* boxes);
			CameraMatrix(Boxes* boxes, CameraMatrix* old);
			CameraMatrix(Boxes* boxes, cv::Matx34d matrix);
			~CameraMatrix();

			cv::Matx34d matrix;
			PointCloud* point_cloud;
			double reprojection_error;

			// Methods
			cv::Mat get_rotation_matrix() const;
			cv::Mat get_translation_vector() const;

			double get_rotation_determinant() const;
			bool rotation_is_coherent() const;
			double percentage_of_points_in_front_of_camera() const;

		private:
			Boxes* boxes = NULL;

			void init(Boxes* boxes, cv::Matx34d matrix);
	};
};

#endif
