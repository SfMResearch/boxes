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

#ifndef BOXES_UTIL_H
#define BOXES_UTIL_H

#ifdef BOXES_PRIVATE

#include <string>

#endif

namespace Boxes {

#ifdef BOXES_PRIVATE

	std::string filename_implant_counter(const std::string filename, int counter);
	std::pair<std::string, std::string> split_once(const std::string what, const std::string delimiter);
	std::string strip(std::string s);
	std::string tolower(const std::string s);

#endif

};

#endif
