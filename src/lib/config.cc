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

#include <fstream>
#include <string>

#include <boxes/config.h>
#include <boxes/constants.h>
#include <boxes/util.h>

namespace Boxes {
	Config::Config() {
		// Initialize with default settings.
		this->set("FEATURE_DETECTOR",           DEFAULT_FEATURE_DETECTOR);
		this->set("FEATURE_DETECTOR_EXTRACTOR", DEFAULT_FEATURE_DETECTOR_EXTRACTOR);
		this->set("MATCH_VALID_RATIO",		DEFAULT_MATCH_VALID_RATIO);
		this->set("EPIPOLAR_DISTANCE_FACTOR",   DEFAULT_EPIPOLAR_DISTANCE_FACTOR);

#ifdef BOXES_NONFREE
		this->set("SURF_MIN_HESSIAN",           DEFAULT_SURF_MIN_HESSIAN);
#endif
	}

	void Config::dump() const {
		std::cout << "Configuration:" << std::endl;

		for (std::map<std::string, std::string>::const_iterator i = this->map.begin(); i != this->map.end(); i++) {
			std::cout << "\t" << i->first << " = " << i->second << std::endl;
		}

		std::cout << std::endl;
	}

	std::string Config::get(std::string key) {
		return this->map[key];
	}

	int Config::get_int(std::string key) {
		int ret;

		std::istringstream value(this->get(key));
		value >> ret;

		return ret;
	}

	double Config::get_double(std::string key) {
		double ret;

		std::istringstream value(this->get(key));
		value >> ret;

		return ret;
	}

	bool Config::get_bool(std::string key) {
		std::string val = this->get(key);

		// Convert to lowercase for easy comparison.
		val = tolower(val);

		if ((val == "true") || (val == "yes") || (val == "1"))
			return true;

		return false;
	}

	void Config::set(std::string key, std::string value) {
		this->map[key] = value;
	}

	void Config::read(const std::string filename) {
		std::string line;

		std::ifstream file(filename);
		if (!file.is_open())
			return;

		while (std::getline(file, line)) {
			this->parse_line(line);
		}

		file.close();
	}

	void Config::parse_line(const std::string line) {
		std::pair<std::string, std::string> args = split_once(line, "=");

		std::string key = strip(args.first);
		std::string val = strip(args.second);

		assert(!key.empty());
		assert(!val.empty());

		this->set(key, val);
	}
}
