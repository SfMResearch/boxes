
#include <sstream>
#include <string>

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
}
