
#include <string>

#include <boxes.h>
#include "tests.h"

int main() {
	Boxes::Boxes boxes;

	Boxes::Image* image1;
	Boxes::Image* image2;

	// Load two images to work with...
	boxes.img_read(IMG1);
	boxes.img_read(IMG2);

	// Try loading an non-existing image...
	image1 = boxes.img_get(42);
	assert(image1 == NULL);

	// Get the both existing images...
	image1 = boxes.img_get(0);
	assert(image1 != NULL);

	image2 = boxes.img_get(1);
	assert(image2 != NULL);

	// Make sure that they are both no the same...
	assert(image1 != image2);

	exit(0);
}
