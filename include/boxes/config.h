
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
