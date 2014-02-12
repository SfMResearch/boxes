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

#ifndef BOXES_MULTI_CAMERA_H
#define BOXES_MULTI_CAMERA_H

#include <vector>

#include <boxes/feature_matcher.h>
#include <boxes/image.h>
#include <boxes/multi_camera.h>
#include <boxes/point_cloud.h>

namespace Boxes {
	class MultiCamera {
		public:
			MultiCamera(Boxes* boxes);
			~MultiCamera();

			void show(bool show_convex_hull = false, bool transparent = true) const;

			void add_images(Image* first, Image* second);
			std::pair<Image*, Image*> get_image_pair(unsigned int pair_index) const;

			FeatureMatcher* get_feature_matcher(unsigned int index) const;

			void run(bool use_optical_flow);

			void write_matches_all(const std::string* filename) const;
			void write_nurbs_all(const std::string* filename) const;
			void write_matches_one(const std::string* filename, unsigned int pair_index) const;

			void write_disparity_map_all(const std::string* filename) const;
			void write_disparity_map_one(const std::string* filename, unsigned int pair_index) const;

			void write_depths_map_all(const std::string* filename) const;
			void write_depths_map_one(const std::string* filename, unsigned int pair_index) const;

			PointCloud* get_point_cloud() const;

			double mean_reprojection_error;

		protected:
			Boxes* boxes = NULL;

			std::vector<FeatureMatcher*> feature_matchers;

			std::vector<Image*> images;
			void add_image(Image* image);

			std::vector<std::pair<Image*, Image*>> image_pairs;
			PointCloud* point_cloud;

			FeatureMatcher* match(Image* image1, Image* image2, bool optical_flow) const;

			std::pair<pcl::PolygonMesh, std::pair<pcl::PointXYZ, pcl::PointXYZ>>
				make_camera_polygon(Image* image, uint8_t r, uint8_t g, uint8_t b, double s) const;
	};
};

#endif
