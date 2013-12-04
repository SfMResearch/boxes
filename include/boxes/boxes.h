
#ifndef BOXES_BOXES_H
#define BOXES_BOXES_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include <boxes/config.h>
#include <boxes/image.h>

namespace Boxes {
	class Boxes {
		public:
			Boxes();
			~Boxes();

			Config* config = NULL;

			// Image operations
			unsigned int img_read(const std::string filename, const std::string resolution = "");
			Image* img_get(unsigned int index);
			unsigned int img_size() const;

			void set_algorithms(const std::string algorithm);

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
