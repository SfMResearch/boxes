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

#include <locale>
#include <sstream>
#include <string>

#include <boxes/constants.h>
#include <boxes/util.h>

namespace Boxes {
	std::string filename_implant_counter(const std::string filename, int counter) {
		std::ostringstream stream;

		std::pair<std::string, std::string> str = split_once(filename, ".");

		stream << str.first << "-" << counter;
		if (!str.second.empty())
			stream << "." << str.second;

		std::cout << stream.str() << std::endl;
		return stream.str();
	}

	std::pair<std::string, std::string> split_once(const std::string what, const std::string delimiter) {
		std::size_t pos = what.find_last_of(delimiter);

		// If the delimiter could not be found, just return the first argument.
		if (pos == std::string::npos) {
			return std::make_pair(what, "");
		}

		std::string string_a = what.substr(0, pos);
		std::string string_b = what.substr(pos + 1, what.size());

		return std::make_pair(string_a, string_b);
	}

	std::string strip(std::string s) {
		size_t pos = s.find_first_not_of(WHITESPACE);
		s.erase(0, pos);

		pos = s.find_last_not_of(WHITESPACE);
		if (std::string::npos != pos)
			s.erase(pos + 1);

		return s;
	}

	std::string tolower(const std::string s) {
		std::string result;

		std::locale loc;
		for (unsigned int i = 0; i < s.length(); i++) {
			result += std::tolower(s.at(i), loc);
		}

		return result;
	}
}
