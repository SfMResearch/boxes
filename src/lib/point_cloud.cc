
#include <boxes/point_cloud.h>
#include <boxes/structs.h>

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
}
