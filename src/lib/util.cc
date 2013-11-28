
#include <locale>
#include <sstream>
#include <string>

#include <boxes/constants.h>
#include <boxes/util.h>

namespace Boxes {
	std::string filename_implant_counter(const std::string* filename, int counter) {
		std::ostringstream stream;

		// Replace extension by nurbs file extension.
		std::size_t pos = filename->find_last_of(".");
		if (pos != std::string::npos) {
			stream << std::string(filename->substr(0, pos));
			stream << "-" << counter;
			stream << std::string(filename->substr(pos, filename->size()));
		} else {
			stream << *filename;
			stream << "-" << counter;
		}

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
