
#include <string>

#include <boxes.h>
#include "tests.h"

int main() {
	Boxes::Boxes boxes;
	unsigned int img_idx;

	// Load first image...
	img_idx = boxes.img_read(IMG1);
	std::cout << "Index of the image that has been loaded first: " << img_idx << std::endl;
	assert(img_idx == 0);

	// Load a second image...
	img_idx = boxes.img_read(IMG2);
	std::cout << "Index of the image that has been loaded second: " << img_idx << std::endl;
	assert(img_idx == 1);

	// Check how many images have been loaded...
	assert(boxes.img_size() == 2);

	exit(0);
}
