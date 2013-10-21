
#ifndef BOXES_CONVERTERS_H
#define BOXES_CONVERTERS_H

#ifdef BOXES_PRIVATE

#include <opencv2/opencv.hpp>
#include <vector>

#include <boxes/suppress_warnings.h>
INCLUDE_IGNORE_WARNINGS_BEGIN
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
INCLUDE_IGNORE_WARNINGS_END
#endif

namespace Boxes {

#ifdef BOXES_PRIVATE

// Convert degree to rad.
#ifndef DEG2RAD
#define DEG2RAD(x) (M_PI / (180 * x))
#endif

#define BOXES_EPSILON 10e-9

#define IS_ZERO(x) (fabsf(x) <= BOXES_EPSILON)
#define GREATER_OR_EQUAL_TO_ZERO(x) (x >= -BOXES_EPSILON)

	// KeyPoints to Points
	inline std::vector<cv::Point2f> convertKeyPoints(const std::vector<cv::KeyPoint>* keypoints) {
		std::vector<cv::Point2f> points;

		for (std::vector<cv::KeyPoint>::const_iterator i = keypoints->begin(); i != keypoints->end(); ++i) {
			points.push_back(i->pt);
		}

		return points;
	}

	// PointCloud<PointXYZRGB> to PointCloud<PointXYZ>
	inline pcl::PointCloud<pcl::PointXYZ>::Ptr convertPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

		for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator i = cloud_in->begin(); i != cloud_in->end(); ++i) {
			pcl::PointXYZ point = pcl::PointXYZ(i->x, i->y, i->z);
			cloud_out->push_back(point);
		}

		return cloud_out;
	}
#endif

};

#endif
