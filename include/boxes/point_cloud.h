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

#ifndef BOXES_POINT_CLOUD_H
#define BOXES_POINT_CLOUD_H

#include <boxes/suppress_warnings.h>
INCLUDE_IGNORE_WARNINGS_BEGIN
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
INCLUDE_IGNORE_WARNINGS_END

#include <string>
#include <vector>

#include <boxes/boxes.h>
#include <boxes/cloud_point.h>
#include <boxes/forward_declarations.h>
#include <boxes/image.h>

#define POINT_CLOUD_USE_STATISTICAL_OUTLIER_REMOVAL

namespace Boxes {
	class PointCloud {
		public:
			PointCloud(Boxes* boxes);
			~PointCloud();

			void add_point(CloudPoint point);
			void remove_point(const CloudPoint* point);
			void cut_curve(const Image* image);
			const std::vector<CloudPoint>* get_points() const;

			// Iterator implementation
			std::vector<CloudPoint>::iterator begin();
			std::vector<CloudPoint>::const_iterator begin() const;
			std::vector<CloudPoint>::iterator end();
			std::vector<CloudPoint>::const_iterator end() const;

			void merge(const PointCloud* other);
			void clear();

			void write(const std::string filename) const;

			unsigned int size() const;

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr generate_pcl_point_cloud() const;
			void write_depths_map(std::string filename, Image* image) const;

			void show() const;

			// Convex hull
			const pcl::ConvexHull<pcl::PointXYZRGB>* get_convex_hull();
			const pcl::PolygonMesh* get_convex_hull_mesh();
			void write_convex_hull(const std::string filename);

			// Volume
			double get_volume();
			void set_scale(double scale);

		private:
			Boxes* boxes = NULL;

			double scale = 1;
			std::vector<CloudPoint> points;

			// Convex hull
			pcl::ConvexHull<pcl::PointXYZRGB>* convex_hull = NULL;
			pcl::PolygonMesh* convex_hull_mesh = NULL;
			void compute_convex_hull(pcl::ConvexHull<pcl::PointXYZRGB>* convex_hull, pcl::PolygonMesh* convex_hull_mesh) const;
			void reset_convex_hull();
	};
};

#endif
