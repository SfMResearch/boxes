
#include <getopt.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <boxes.h>

int main(int argc, char **argv) {
	Boxes::Boxes boxes;

	std::string output;
	bool use_optical_flow = false;
	bool visualize = false;

	while (1) {
		static struct option long_options[] = {
			{"optical-flow", no_argument,        0, 'O'},
			{"output",       required_argument,  0, 'o'},
			{"version",      no_argument,        0, 'V'},
			{"visualize",    no_argument,        0, 'v'},
			{0, 0, 0, 0}
		};
		int option_index = 0;

		int c = getopt_long(argc, argv, "Oo:Vv", long_options, &option_index);

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

			case 'O':
				use_optical_flow = true;
				break;

			case 'o':
				output.assign(optarg);
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

	// Warn if not enough images have been loaded.
	if (boxes.img_size() < 2) {
		std::cerr << "You need to load at least two image files! Exiting." << std::endl;
		exit(2);
	}

	Boxes::BoxesImage* image1 = boxes.img_get(0);
	Boxes::BoxesImage* image2 = boxes.img_get(1);

	Boxes::FeatureMatcher* matcher = NULL;
	if (use_optical_flow) {
		matcher = boxes.match(image1, image2);
	} else {
		matcher = boxes.match_optical_flow(image1, image2);
	}
	assert(matcher);

	// Run it.
	matcher->run();

	if (matcher->best_camera_matrix) {
		std::cout << matcher->best_camera_matrix->matrix << std::endl;

		if (visualize)
			matcher->visualize_point_cloud(matcher->best_camera_matrix);
	}

	if (!output.empty()) {
		std::cout << "Writing matched image to " << output << "..." << std::endl;
		matcher->draw_matches(output);
	}

	// Free memory.
	delete matcher;

	exit(0);
}
