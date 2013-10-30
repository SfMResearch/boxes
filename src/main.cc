
#include <getopt.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <boxes.h>

int main(int argc, char **argv) {
	Boxes::Boxes boxes;

	std::string output;
	std::string output_depths_map;
	std::string output_disparity_map;
	std::string output_mesh;
	std::string output_hull;

	bool use_optical_flow = false;
	bool visualize = false;

	while (1) {
		static struct option long_options[] = {
			{"depths-map",    required_argument,  0, 'd'},
			{"disparity-map", required_argument,  0, 'D'},
			{"mesh",          required_argument,  0, 'm'},
			{"convex-hull",   required_argument,  0, 'c'},
			{"optical-flow",  no_argument,        0, 'O'},
			{"output",        required_argument,  0, 'o'},
			{"version",       no_argument,        0, 'V'},
			{"visualize",     no_argument,        0, 'v'},
			{0, 0, 0, 0}
		};
		int option_index = 0;

		int c = getopt_long(argc, argv, "c:D:d:m:Oo:Vv", long_options, &option_index);

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
				output_disparity_map.assign(optarg);
				break;

			case 'd':
				output_depths_map.assign(optarg);
				break;

			case 'm':
				output_mesh.assign(optarg);
				break;

			case 'c':
				output_hull.assign(optarg);
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

	Boxes::Image* image1 = boxes.img_get(0);
	Boxes::Image* image2 = boxes.img_get(1);

	Boxes::FeatureMatcher* matcher = NULL;
	if (use_optical_flow) {
		matcher = boxes.match_optical_flow(image1, image2);
	} else {
		matcher = boxes.match(image1, image2);
	}
	assert(matcher);

	// Run it.
	Boxes::CameraMatrix* camera_matrix = matcher->run();

	if (!camera_matrix) {
		std::cerr << "Could not find a suitable camera matrix. Exiting." << std::endl;
		exit(1);
	}

	// First, write everything to file.
	if (!output.empty()) {
		std::cout << "Writing matched image to " << output << "..." << std::endl;
		matcher->draw_matches(output);
	}

	// Delete the matcher, which is not needed any more.
	delete matcher;

	if (!output_depths_map.empty()) {
		std::cout << "Writing best depths map to " << output_depths_map << "..." << std::endl;
		camera_matrix->point_cloud.write_depths_map(output_depths_map, image1);
	}

	if (!output_disparity_map.empty()) {
		std::cout << "Writing disparity map to " << output_disparity_map << "..." << std::endl;

		Boxes::Image* disparity_map = image1->get_disparity_map(image2);
		if (disparity_map) {
			disparity_map->write(output_disparity_map);
			delete disparity_map;
		}
	}

	/* Strip all points from the point cloud, if they are not within the
	 * NURBS curve (if that one is available).
	 */
	Boxes::PointCloud point_cloud;
	if (image1->has_curve()) {
		point_cloud = image1->cut_out_curve(&camera_matrix->point_cloud);
	} else {
		point_cloud = camera_matrix->point_cloud;
	}

	// Calculate mesh
	pcl::PolygonMesh* mesh = point_cloud.triangulate(image1);
	pcl::PolygonMesh* mesh_hull = new pcl::PolygonMesh();
	pcl::ConvexHull<pcl::PointXYZRGB>* hull = new pcl::ConvexHull<pcl::PointXYZRGB>();
	point_cloud.generate_convex_hull(hull, mesh_hull);

	if (!output_mesh.empty()) {
		std::cout << "Writing mesh to " << output_mesh << "..." << std::endl;
		point_cloud.write_polygon_mesh(output_mesh, mesh);
	}

	if (!output_hull.empty()) {
		std::cout << "Writing convex hull to " << output_hull << "..." << std::endl;
		point_cloud.write_polygon_mesh(output_hull, mesh_hull);
	}

	// Print estimated volume.
	std::cout << "Estimated volume: " << hull->getTotalVolume() << std::endl;

	if (visualize)
		point_cloud.visualize_point_cloud(image1);

	std::cout << camera_matrix->matrix << std::endl;

	// Free memory.
	delete camera_matrix;

	if (mesh)
		delete mesh;

	if (mesh_hull)
		delete mesh_hull;

	if (hull)
		delete hull;

	exit(0);
}
