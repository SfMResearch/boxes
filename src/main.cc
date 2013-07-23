
#include <iostream>

#include <boxes.h>

int main(int argc, char **argv) {
	Boxes::Boxes boxes;

	std::cout << boxes.version_string() << std::endl;
}
