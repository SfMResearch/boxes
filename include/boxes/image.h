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

#ifndef BOXES_IMAGE_H
#define BOXES_IMAGE_H

#include <opencv2/opencv.hpp>
#include <map>
#include <string>
#include <vector>

#include <boxes/forward_declarations.h>
#include <boxes/boxes.h>
#include <boxes/camera_matrix.h>
#include <boxes/constants.h>
#include <boxes/point_cloud.h>

#include <moges/Types.h>
#include <moges/NURBS/Curve.h>

namespace Boxes {
	class Image {
		public:
			Image(Boxes* boxes);
			Image(Boxes* boxes, const std::string filename, int width = -1, int height = -1);
			Image(Boxes* boxes, cv::Mat mat);
			void init(Boxes* boxes);
			~Image();

			// Basic methods.
			void show();
			void write(const std::string filename);
			cv::Size size() const;

			std::string filename;

			// mat
			const cv::Mat* get_mat() const;
			const cv::Mat* get_mat(int code) const;
			const cv::Mat* get_greyscale_mat() const;

			// descriptors
			cv::Mat* get_descriptors(std::vector<cv::KeyPoint>* keypoints) const;
			cv::Mat* get_descriptors(std::vector<cv::KeyPoint>* keypoints, const std::string detector_type) const;

			// keypoints
			std::vector<cv::KeyPoint>* get_keypoints();
			std::vector<cv::KeyPoint>* get_keypoints(const std::string detector_type);

			// (good) features
			std::vector<cv::Point2f> get_good_features_to_track(int max_corners = 1500, double quality_level = 0.05,
				double min_distance = 2.0);

			// distance
			void set_distance(unsigned int distance);
			unsigned int get_distance();

			// camera matrix
			cv::Mat get_camera() const;
			std::string find_camera_file() const;

			// disparity map
			Image* get_disparity_map(Image *other_img);

			// curve
			bool has_curve() const;
			std::vector<cv::Point2f> discretize_curve() const;
			cv::Mat draw_curve();

			CameraMatrix* camera_matrix = NULL;
			void update_camera_matrix(CameraMatrix* camera_matrix);

		private:
			Boxes* boxes = NULL;

			cv::Mat mat;
			void decode_jfif_data(std::string filename);

			std::string find_file_with_extension(const std::string filename, const std::string extension) const;

			//scaling
			double scaling = -1;

			// camera matrix
			cv::Mat camera;
			cv::Mat read_camera(const std::string filename) const;
			cv::Mat guess_camera() const;

			// curve
			MoGES::NURBS::Curve* curve = NULL;
			MoGES::NURBS::Curve* read_curve(const std::string filename) const;
			std::string find_curve_file() const;

			// distance
			unsigned int distance = 0;

			// keypoint cache
			std::map<std::string, std::vector<cv::KeyPoint>*> keypoints;
			std::vector<cv::KeyPoint>* compute_keypoints(const std::string detector_type = DEFAULT_FEATURE_DETECTOR) const;
	};
};

#endif
