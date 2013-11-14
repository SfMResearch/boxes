
#ifndef BOXES_BOXES_H
#define BOXES_BOXES_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include <boxes/image.h>

namespace Boxes {
	class Boxes {
		public:
			Boxes();

			// Image operations
			unsigned int img_read(const std::string filename);
			Image* img_get(unsigned int index);
			unsigned int img_size() const;

			std::string version_string() const;

			// Iterator
			std::vector<Image*>::iterator begin();
			std::vector<Image*>::const_iterator begin() const;
			std::vector<Image*>::iterator end();
			std::vector<Image*>::const_iterator end() const;

			std::vector<std::pair<Image*, Image*>> make_pairs() const;

		private:
			std::vector<Image*> images;
	};
}

#endif
