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

	Boxes::Image* image1;
	Boxes::Image* image2;

	// Load two images to work with...
	boxes.img_read(IMG1);
	boxes.img_read(IMG2);

	// Try loading an non-existing image...
	image1 = boxes.img_get(42);
	assert(image1 == NULL);

	// Get the both existing images...
	image1 = boxes.img_get(0);
	assert(image1 != NULL);

	image2 = boxes.img_get(1);
	assert(image2 != NULL);

	// Make sure that they are both no the same...
	assert(image1 != image2);

	exit(0);
}
