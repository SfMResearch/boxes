
#include <getopt.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <boxes.h>

int main(int argc, char **argv) {
	Boxes::Boxes boxes;

	std::string output_depths_maps;
	std::string output_disparity_maps;
	std::string output_matches;
	std::string output_hull;

	bool use_optical_flow = false;
	bool visualize = false;

	while (1) {
		static struct option long_options[] = {
			{"depths-maps",    required_argument,  0, 'd'},
			{"disparity-maps", required_argument,  0, 'D'},
			{"matches",        required_argument,  0, 'm'},
			{"convex-hull",    required_argument,  0, 'c'},
			{"optical-flow",   no_argument,        0, 'O'},
			{"version",        no_argument,        0, 'V'},
			{"visualize",      no_argument,        0, 'v'},
			{0, 0, 0, 0}
		};
		int option_index = 0;

		int c = getopt_long(argc, argv, "c:D:d:m:OVv", long_options, &option_index);

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

			case 'D':
				output_disparity_maps.assign(optarg);
				break;

			case 'd':
				output_depths_maps.assign(optarg);
				break;

			case 'm':
				output_matches.assign(optarg);
				break;

			case 'c':
				output_hull.assign(optarg);
				break;

			case 'O':
				use_optical_flow = true;
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

	Boxes::MultiCamera multi_camera;

	// Add all images to the multi camera environment
	for (std::pair<Boxes::Image*, Boxes::Image*> image_pair: boxes.make_pairs()) {
		multi_camera.add_images(image_pair.first, image_pair.second);
	}

	multi_camera.run(use_optical_flow);

	if (!output_matches.empty()) {
		std::cout << "Writing matches..." << std::endl;
		multi_camera.write_matches_all(&output_matches);
	}

	if (!output_depths_maps.empty()) {
		std::cout << "Writing depths maps..." << std::endl;
		multi_camera.write_depths_map_all(&output_depths_maps);
	}

	if (!output_disparity_maps.empty()) {
		std::cout << "Writing disparity maps..." << std::endl;
		multi_camera.write_disparity_map_all(&output_disparity_maps);
	}

	Boxes::PointCloud* point_cloud = multi_camera.get_point_cloud();

	if (!output_hull.empty()) {
		std::cout << "Writing convex hull to " << output_hull << "..." << std::endl;
		point_cloud->write_convex_hull(output_hull);
	}

	// Print estimated volume.
	std::cout << "Estimated volume: " << point_cloud->get_volume() << std::endl;

	if (visualize)
		multi_camera.show();

	exit(0);
}
