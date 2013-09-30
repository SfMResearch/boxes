
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define IMG1 "images/test-square-1.jpg"
#define IMG2 "images/test-square-2.jpg"

static void test_init() {
	/*
	 * Initialize the test environment.
	 */
	int err;

	// Change into the test directory.
	char* test_dir = getenv("TEST_DIR");
	if (test_dir == NULL) {
		fprintf(stderr, "TEST_DIR is not set\n");
		exit(1);
	}

	err = chdir(test_dir);
	if (err == 0) {
		printf("Successfully changed to '%s'\n", test_dir);
	} else {
		fprintf(stderr, "Could not chdir to '%s': %s\n", test_dir, strerror(errno));
		exit(1);
	}

	printf("Test environment has been successfully initialized.");
}

#define TEST_INIT test_init();
