
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

#include <boxes/cloud_point.h>
#include <boxes/forward_declarations.h>
#include <boxes/image.h>
#include <boxes/structs.h>

#define POINT_CLOUD_USE_STATISTICAL_OUTLIER_REMOVAL

namespace Boxes {
	class PointCloud {
		public:
			PointCloud();
			~PointCloud();

			void add_point(CloudPoint point);
			void remove_point(const CloudPoint* point);
			const std::vector<CloudPoint>* get_points() const;

			// Iterator implementation
			std::vector<CloudPoint>::iterator begin();
			std::vector<CloudPoint>::const_iterator begin() const;
			std::vector<CloudPoint>::iterator end();
			std::vector<CloudPoint>::const_iterator end() const;

			void merge(const PointCloud* other);

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

		private:
			std::vector<CloudPoint> points;

			// Convex hull
			pcl::ConvexHull<pcl::PointXYZRGB>* convex_hull = NULL;
			pcl::PolygonMesh* convex_hull_mesh = NULL;
			void compute_convex_hull(pcl::ConvexHull<pcl::PointXYZRGB>* convex_hull, pcl::PolygonMesh* convex_hull_mesh) const;
			void reset_convex_hull();
	};
};

#endif
