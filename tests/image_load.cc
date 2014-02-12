/***
	This file is part of the boxes library.

	Copyright (C) 2013-2014  Christian Bodenstein, Michael Tremer

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
***/

#include <string>

#include <boxes.h>
#include "tests.h"

int main() {
	TEST_INIT

	Boxes::Boxes boxes;
	unsigned int img_idx;

	// Load first image...
	img_idx = boxes.img_read(IMG1);
	std::cout << "Index of the image that has been loaded first: " << img_idx << std::endl;
	assert(img_idx == 0);

	// Load a second image...
	img_idx = boxes.img_read(IMG2);
	std::cout << "Index of the image that has been loaded second: " << img_idx << std::endl;
	assert(img_idx == 1);

	// Check how many images have been loaded...
	assert(boxes.img_size() == 2);

	exit(0);
}
