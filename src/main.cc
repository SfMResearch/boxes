
#include <getopt.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <boxes.h>

int main(int argc, char **argv) {
	Boxes::Boxes boxes;

	bool visualize = false;

	while (1) {
		static struct option long_options[] = {
			{"version",     no_argument,        0, 'V'},
			{"visualize",   no_argument,        0, 'v'},
			{0, 0, 0, 0}
		};
		int option_index = 0;

		int c = getopt_long(argc, argv, "V", long_options, &option_index);

		if (c == -1)
			break;

		switch (c) {
			case 0:
				if (long_options[option_index].flag != 0)
					break;

				std::cerr << "option: " << long_options[option_index].name;
				if (optarg)
					std::cerr << " with argument " << optarg;
				std::cerr << std::endl;
				break;

			case 'V':
				std::cout << boxes.version_string() << std::endl;
				exit(0);
				break;

			case 'v':
				visualize = true;
				break;

			case '?':
				// getopt_long already printed an error message.
				break;

			default:
				abort();
		}
	}

	while (optind < argc) {
		std::string filename = argv[optind++];

		std::cout << "Reading image file " << filename << "..." << std::endl;
		boxes.img_read(filename);
	}

	Boxes::BoxesFeatureMatcher* matcher = boxes.match(0, 1);

	if (matcher->best_camera_matrix) {
		std::cout << matcher->best_camera_matrix->matrix << std::endl;

		if (visualize)
			matcher->visualize_point_cloud(matcher->best_camera_matrix);
	}

	exit(0);
}
