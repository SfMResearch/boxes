
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

#include <boxes/image.h>
#include <boxes/structs.h>

#define POINT_CLOUD_USE_STATISTICAL_OUTLIER_REMOVAL

namespace Boxes {
	class PointCloud {
		public:
			PointCloud();

			unsigned int size() const;

			std::vector<CloudPoint> points;
			void add_point(CloudPoint point);

			pcl::PolygonMesh triangulate(const Image* image) const;
			void write_polygon_mesh(std::string filename, pcl::PolygonMesh* mesh) const;

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr generate_pcl_point_cloud(const Image* image = NULL) const;
			void write_depths_map(std::string filename, Image* image) const;

		private:
			pcl::PointCloud<pcl::Normal>::Ptr estimate_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const;
	};
};

#endif
