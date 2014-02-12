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

	// Get version of the linked library and print string.
	std::string version = boxes.version_string();
	std::cout << version << std::endl;

	if (version.size() > 0)
		exit(0);

	exit(1);
}
