/***
	This file is part of the boxes library.

	Copyright (C) 2013-2014  Christian Bodenstein, Michael Tremer

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Lesser General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU Lesser General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
***/

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
