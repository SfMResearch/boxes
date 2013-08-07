
#ifndef BOXES_CONVERTERS_H
#define BOXES_CONVERTERS_H

#ifdef BOXES_PRIVATE
#include <pcl/point_types.h>
#endif

namespace Boxes {

#ifdef BOXES_PRIVATE

// Convert degree to rad.
#define DEG2RAD(x) (M_PI / (180 * x))

	inline pcl::PointCloud<pcl::PointXYZ>::Ptr convertPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

		for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator i = cloud_in->begin(); i != cloud_in->end(); i++) {
			pcl::PointXYZ point = pcl::PointXYZ(i->x, i->y, i->z);
			cloud_out->push_back(point);
		}

		return cloud_out;
	}
#endif

};

#endif
