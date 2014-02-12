/***
	This file is part of the boxes library.

	Copyright (C) 2013-2014  Christian Bodenstein, Michael Tremer

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
***/

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
