
#ifndef BOXES_BOXES_H
#define BOXES_BOXES_H

#include <list>
#include <string>
#include <vector>

#include <boxes/image.h>

namespace Boxes {
	class Boxes {
		public:
			Boxes();

			// Image operations
			unsigned int img_read(const std::string filename);
			BoxesImage* img_get(unsigned int index);

			std::vector<cv::DMatch> calc_matches(unsigned int index1, unsigned int index2);

			std::string version_string() const;

		private:
			std::list<BoxesImage*> images;
	};
}

#endif
