
#include <boxes/boxes.h>

/*
 * Contructor.
 */
Boxes::Boxes() {

}

/*
 *
 */
string Boxes::version_string() const {
	string str;
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
