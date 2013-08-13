
#ifndef BOXES_BOXES_H
#define BOXES_BOXES_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include <boxes/feature_matcher.h>
#include <boxes/feature_matcher_optical_flow.h>
#include <boxes/image.h>

namespace Boxes {
	class Boxes {
		public:
			Boxes();

			// Image operations
			unsigned int img_read(const std::string filename);
			BoxesImage* img_get(unsigned int index);
			unsigned int img_size() const;

			FeatureMatcher* match(BoxesImage* image1, BoxesImage* image2);
			FeatureMatcherOpticalFlow* match_optical_flow(BoxesImage* image1, BoxesImage* image2);

			std::string version_string() const;

		private:
			std::vector<BoxesImage*> images;
	};
}

#endif
