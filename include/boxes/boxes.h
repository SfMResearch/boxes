/***
	This file is part of the boxes library.

	Copyright (C) 2013-2014  Christian Bodenstein, Michael Tremer

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Lesser General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU Lesser General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
***/

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
