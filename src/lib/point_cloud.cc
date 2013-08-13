
#include <boxes/suppress_warnings.h>
INCLUDE_IGNORE_WARNINGS_BEGIN
#include <pcl/features/normal_3d.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
INCLUDE_IGNORE_WARNINGS_END

#include <boxes/constants.h>
#include <boxes/converters.h>
#include <boxes/point_cloud.h>
#include <boxes/structs.h>

INCLUDE_IGNORE_WARNINGS_BEGIN
#ifdef POINT_CLOUD_USE_STATISTICAL_OUTLIER_REMOVAL
#include <pcl/filters/statistical_outlier_removal.h>
#endif
INCLUDE_IGNORE_WARNINGS_END

namespace Boxes {
	/*
	 * Contructor.
	 */
	PointCloud::PointCloud() {
	}

	unsigned int PointCloud::size() const {
		return this->points.size();
	}

	void PointCloud::add_point(CloudPoint point) {
		this->points.push_back(point);
	}

	pcl::PolygonMesh PointCloud::triangulate(const Image* image) const {
		// Concert point cloud into PCL format.
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb = this->generate_pcl_point_cloud(image);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = convertPointCloud(cloud_rgb);

		// Normal estimation
		pcl::PointCloud<pcl::Normal>::Ptr normals = this->estimate_normals(cloud);

		// Concatenate points and normal fields.
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
		pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

		// Create search tree
		pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
		tree->setInputCloud(cloud_with_normals);

		// Initialize triangulation
		pcl::GreedyProjectionTriangulation<pcl::PointNormal> pt = pcl::GreedyProjectionTriangulation<pcl::PointNormal>();
		pcl::PolygonMesh triangles;

		// Set the maximum distance between connected points (maximum edge length)
		pt.setSearchRadius(POINT_CLOUD_TRIANGULATION_SEARCH_RADIUS);

		// Set typical values for the parameters
		pt.setMu(POINT_CLOUD_TRIANGULATION_MULTIPLIER);
		pt.setMaximumNearestNeighbors(POINT_CLOUD_TRIANGULATION_MAX_NEAREST_NEIGHBOUR);
		pt.setMaximumSurfaceAngle(POINT_CLOUD_TRIANGULATION_MAX_SURFACE_ANGLE);
		pt.setMinimumAngle(POINT_CLOUD_TRIANGULATION_MIN_ANGLE);
		pt.setMaximumAngle(POINT_CLOUD_TRIANGULATION_MAX_ANGLE);
		pt.setNormalConsistency(false);

		// Get result
		pt.setInputCloud(cloud_with_normals);
		pt.setSearchMethod(tree);
		pt.reconstruct(triangles);

		return triangles;
	}

	void PointCloud::write_polygon_mesh(std::string filename, pcl::PolygonMesh* mesh) const {
		pcl::io::saveVTKFile(filename, *mesh);
	}

	pcl::PointCloud<pcl::Normal>::Ptr PointCloud::estimate_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const {
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(cloud);

		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
		normal_estimation.setInputCloud(cloud);
		normal_estimation.setSearchMethod(tree);
		normal_estimation.setKSearch(20);
		normal_estimation.compute(*normals);

		return normals;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloud::generate_pcl_point_cloud(const Image* image) const {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		const cv::Mat* mat = NULL;
		if (image)
			mat = image->get_mat();

		for (std::vector<CloudPoint>::const_iterator i = this->points.begin(); i != this->points.end(); ++i) {
			pcl::PointXYZRGB cloud_point;
			cloud_point.x = i->pt.x;
			cloud_point.y = i->pt.y;
			cloud_point.z = i->pt.z;

			// Apply RGB value from image for this pixel.
			if (mat) {
				cv::Vec3b pixel = mat->at<cv::Vec3b>(i->keypoint.pt.y, i->keypoint.pt.x);
				uint32_t rgb = ((uint32_t)pixel[2] << 16 | (uint32_t)pixel[1] << 8 | (uint32_t)pixel[0]);
				cloud_point.rgb = *reinterpret_cast<float*>(&rgb);

			// Otherwise set them in a bright colour.
			} else {
				cloud_point.rgb = 0xffffff;
			}

			cloud->push_back(cloud_point);
		}

		cloud->width = (uint32_t) cloud->points.size();
		cloud->height = 1;

#ifdef POINT_CLOUD_USE_STATISTICAL_OUTLIER_REMOVAL
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
		sor.setInputCloud(cloud);
		sor.setMeanK(50);
		sor.setStddevMulThresh(1.0);
		sor.filter(*cloud_filtered);

		return cloud_filtered;
#else
		return cloud;
#endif
	}
}
