
#ifndef BOXES_POINT_CLOUD_H
#define BOXES_POINT_CLOUD_H

#include <boxes/suppress_warnings.h>
INCLUDE_IGNORE_WARNINGS_BEGIN
#include <pcl/PolygonMesh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
INCLUDE_IGNORE_WARNINGS_END

#include <string>
#include <vector>

#include <boxes/forward_declarations.h>
#include <boxes/image.h>
#include <boxes/structs.h>

#define POINT_CLOUD_USE_STATISTICAL_OUTLIER_REMOVAL

namespace Boxes {
	class PointCloud {
		public:
			PointCloud();

			void add_point(CloudPoint point);
			const std::vector<CloudPoint>* get_points() const;

			// Iterator implementation
			std::vector<CloudPoint>::iterator begin();
			std::vector<CloudPoint>::const_iterator begin() const;
			std::vector<CloudPoint>::iterator end();
			std::vector<CloudPoint>::const_iterator end() const;

			unsigned int size() const;

			pcl::PolygonMesh triangulate(const Image* image) const;
			void write_polygon_mesh(std::string filename, pcl::PolygonMesh* mesh) const;

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr generate_pcl_point_cloud(const Image* image = NULL) const;
			void write_depths_map(std::string filename, Image* image) const;

			void visualize_point_cloud(const Image* image = NULL);

			double generate_convex_hull(pcl::PolygonMesh &mesh) const;

		private:
			std::vector<CloudPoint> points;

			pcl::PointCloud<pcl::Normal>::Ptr estimate_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const;
	};
};

#endif
