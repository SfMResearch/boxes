
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
