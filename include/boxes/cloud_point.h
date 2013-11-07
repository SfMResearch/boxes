
#ifndef BOXES_CLOUD_POINT_H
#define BOXES_CLOUD_POINT_H

#include <opencv2/opencv.hpp>

#include <boxes/image.h>

namespace Boxes {
	class CloudPoint {
		public:
			CloudPoint();

			cv::Point3d pt;
			cv::KeyPoint keypoint;
			int keypoint_index = -1;
			double reprojection_error;

			bool operator==(const CloudPoint& other) const;

			// colours
			void set_colour_from_image(Image* image);
			float get_colour(float def = 0xffffff) const;

		private:
			float colour = -1.0;
	};
};

#endif
