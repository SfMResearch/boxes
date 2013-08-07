
#include <string>

#include <boxes.h>

int main() {
	Boxes::Boxes boxes;

	// Get version of the linked library and print string.
	std::string version = boxes.version_string();
	std::cout << version << std::endl;

	if (version.size() > 0)
		exit(0);

	exit(1);
}
