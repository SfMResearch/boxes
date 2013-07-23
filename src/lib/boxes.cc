
#include <boxes/boxes.h>

namespace Boxes {

	/*
	 * Contructor.
	 */
	Boxes::Boxes() {
	}

	/*
	 *
	 */
	std::string Boxes::version_string() const {
		std::string str;
		str += PACKAGE_NAME;
		str += " ";
		str += VERSION;
		str += " (";
		str += __DATE__;
		str += " ";
		str += __TIME__;
		str += ")";

		return str;
	}
}
