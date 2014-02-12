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

#ifndef BOXES_CONFIG_H
#define BOXES_CONFIG_H

#include <map>
#include <sstream>
#include <string>

namespace Boxes {
	class Config {
		public:
			Config();

			void dump() const;

			std::string get(std::string key);
			int get_int(std::string key);
			double get_double(std::string key);
			bool get_bool(std::string key);

			void set(std::string key, std::string value);

			void read(const std::string filename);
			void parse_line(const std::string line);

		private:
			std::map<std::string, std::string> map;
	};
}

#endif
