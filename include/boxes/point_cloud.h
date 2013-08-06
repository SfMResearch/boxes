
#ifndef BOXES_POINT_CLOUD_H
#define BOXES_POINT_CLOUD_H

#include <vector>

#include <boxes/constants.h>
#include <boxes/structs.h>

namespace Boxes {
	class PointCloud {
		public:
			PointCloud();

			unsigned int size() const;

			std::vector<CloudPoint> points;
			void add_point(CloudPoint point);
	};
};

#endif
