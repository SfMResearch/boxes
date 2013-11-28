
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

		std::string key = args.first;
		std::string val = args.second;

		assert(!key.empty());
		assert(!val.empty());

		this->set(key, val);
	}
}
