
#ifndef BOXES_BOXES_H
#define BOXES_BOXES_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include <boxes/feature_matcher.h>
#include <boxes/image.h>

namespace Boxes {
	class Boxes {
		public:
			Boxes();

			// Image operations
			unsigned int img_read(const std::string filename);
			BoxesImage* img_get(unsigned int index);

			BoxesFeatureMatcher* match(unsigned int index1, unsigned int index2);

			std::string version_string() const;

		private:
			std::vector<BoxesImage*> images;
	};
}

#endif
