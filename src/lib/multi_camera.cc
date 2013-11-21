
#include <sstream>
#include <vector>

#include <boxes/suppress_warnings.h>
INCLUDE_IGNORE_WARNINGS_BEGIN
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
INCLUDE_IGNORE_WARNINGS_END

#include <boxes/converters.h>
#include <boxes/feature_matcher.h>
#include <boxes/feature_matcher_optical_flow.h>
#include <boxes/multi_camera.h>
#include <boxes/image.h>
#include <boxes/util.h>

namespace Boxes {
	/*
	 * Contructor.
	 */
	MultiCamera::MultiCamera() {
		this->point_cloud = new PointCloud();
	}

	MultiCamera::~MultiCamera() {
		for (FeatureMatcher* feature_matcher: this->feature_matchers)
			delete feature_matcher;

		delete this->point_cloud;
	}

	void MultiCamera::add_images(Image* first, Image* second) {
		std::pair<Image*, Image*> pair = std::make_pair(first, second);
		this->image_pairs.push_back(pair);

		this->add_image(first);
		this->add_image(second);
	}

	void MultiCamera::add_image(Image* image) {
		for (std::vector<Image*>::iterator i = this->images.begin(); i != this->images.end(); i++) {
			if (image == *i)
				return;
		}

		this->images.push_back(image);
	}

	std::pair<Image*, Image*> MultiCamera::get_image_pair(unsigned int pair_index) const {
		if (pair_index >= this->image_pairs.size())
			throw new std::exception();

		return this->image_pairs[pair_index];
	}

	FeatureMatcher* MultiCamera::get_feature_matcher(unsigned int index) const {
		if (index >= this->feature_matchers.size())
			throw new std::exception();

		return this->feature_matchers[index];
	}

	void MultiCamera::run(bool use_optical_flow) {
		// Create feature matchers for each image pair.
		for (std::pair<Image*, Image*> image_pair: this->image_pairs) {
			Image* image1 = image_pair.first;
			Image* image2 = image_pair.second;

			// Match the two images.
			FeatureMatcher* matcher = this->match(image1, image2, use_optical_flow);
			this->feature_matchers.push_back(matcher);
		}

		// Calculate matches in parallel.
		#pragma omp parallel for
		for (unsigned int i = 0; i < this->feature_matchers.size(); i++) {
			FeatureMatcher* matcher = feature_matchers[i];

			matcher->match();
		}

		FeatureMatcher* last_matcher = NULL;

		for (FeatureMatcher* matcher: this->feature_matchers) {
			Image* image1 = matcher->image1;
			Image* image2 = matcher->image2;

			// For the first match, find the best camera matrix of the image
			// pair and initialize the point cloud.
			if (last_matcher == NULL) {
				matcher->calculate_camera_matrix();

				/* Strip all points from the point cloud, if they are not within the
				 * NURBS curve (if that one is available).
				 */
				if (image2->has_curve()) {
					image2->cut_out_curve(matcher->point_cloud);
				}

			} else {
				std::vector<cv::KeyPoint>* keypoints2 = image2->get_keypoints();

				std::vector<cv::Point3f> local_point_cloud;
				std::vector<cv::Point2f> image_points;
				std::vector<int> point_cloud_status(last_matcher->point_cloud->size(), 0);

				for (std::vector<CloudPoint>::iterator i = last_matcher->point_cloud->begin(); i != last_matcher->point_cloud->end(); i++) {
					int keypoint2_index = matcher->find_corresponding_keypoint(i->keypoint_index);

					if (keypoint2_index >= 0) {
						cv::KeyPoint keypoint2 = (*keypoints2)[keypoint2_index];

						local_point_cloud.push_back(i->pt);
						image_points.push_back(keypoint2.pt);
					}
				}

				cv::Mat_<double> rvec;
				cv::Mat_<double> translation;
				std::vector<double> distortion_coeff;
				std::vector<int> inliers;

				cv::solvePnPRansac(local_point_cloud, image_points, image1->get_camera(),
					distortion_coeff, rvec, translation, false, 100, 8.0, 100, inliers);

				cv::Mat_<double> rotation;
				cv::Rodrigues(rvec, rotation);

				// Compose combined rotation and translation matrix.
				cv::Matx34d matrix = merge_rotation_and_translation_matrix(&rotation, &translation);

				CameraMatrix camera_matrix = CameraMatrix(matrix);
				image2->update_camera_matrix(&camera_matrix);

				matcher->triangulate_points();
			}

			last_matcher = matcher;
		}

		for (FeatureMatcher* matcher: this->feature_matchers) {
			this->point_cloud->merge(matcher->point_cloud);
		}
	}

	FeatureMatcher* MultiCamera::match(Image* image1, Image* image2, bool optical_flow) const {
		FeatureMatcher* feature_matcher;

		if (optical_flow)
			feature_matcher = new FeatureMatcherOpticalFlow(image1, image2);
		else
			feature_matcher = new FeatureMatcher(image1, image2);

		return feature_matcher;
	}

	PointCloud* MultiCamera::get_point_cloud() const {
		return this->point_cloud;
	}

	void MultiCamera::write_disparity_map_all(const std::string* filename) const {
		for (unsigned int i = 0; i < this->image_pairs.size(); i++)
			this->write_disparity_map_one(filename, i);
	}

	void MultiCamera::write_disparity_map_one(const std::string* filename, unsigned int pair_index) const {
		std::pair<Image*, Image*> pair = this->get_image_pair(pair_index);

		Boxes::Image* disparity_map = pair.first->get_disparity_map(pair.second);
		if (disparity_map) {
			disparity_map->write(filename_implant_counter(filename, pair_index));
			delete disparity_map;
		}
	}

	void MultiCamera::write_matches_all(const std::string* filename) const {
		for (unsigned int i = 0; i < this->feature_matchers.size(); i++)
			this->write_matches_one(filename, i);
	}

	void MultiCamera::write_matches_one(const std::string* filename, unsigned int pair_index) const {
		FeatureMatcher* matcher = this->get_feature_matcher(pair_index);

		matcher->draw_matches(filename_implant_counter(filename, pair_index));
	}

	void MultiCamera::write_depths_map_all(const std::string* filename) const {
		for (unsigned int i = 0; i < this->feature_matchers.size(); i++)
			this->write_depths_map_one(filename, i);
	}

	void MultiCamera::write_depths_map_one(const std::string* filename, unsigned int pair_index) const {
		FeatureMatcher* matcher = this->get_feature_matcher(pair_index);

		CameraMatrix* camera_matrix = matcher->calculate_camera_matrix();

		if (camera_matrix) {
			camera_matrix->point_cloud.write_depths_map(
				filename_implant_counter(filename, pair_index), matcher->image2);

			delete camera_matrix;
		}
	}

	std::pair<pcl::PolygonMesh, std::pair<pcl::PointXYZ, pcl::PointXYZ>>
			MultiCamera::make_camera_polygon(Image* image, uint8_t r, uint8_t g, uint8_t b, double s) const {
		cv::Mat rotation = image->camera_matrix->get_rotation_matrix();
		cv::Mat translation = image->camera_matrix->get_translation_vector();

		cv::Mat position = -(rotation.t() * translation).t();

		cv::Mat v_x =  normalize(rotation.row(0)) * s;
		cv::Mat v_y = -normalize(rotation.row(1)) * s;
		cv::Mat v_z =  normalize(rotation.row(2)) * s;

		std::vector<cv::Mat> points;
		points.push_back(position);

		points.push_back(position + v_z + (v_x / 2.0) + (v_y / 2.0));
		points.push_back(position + v_z + (v_x / 2.0) - (v_y / 2.0));
		points.push_back(position + v_z - (v_x / 2.0) + (v_y / 2.0));
		points.push_back(position + v_z - (v_x / 2.0) - (v_y / 2.0));

		pcl::PointCloud<pcl::PointXYZRGB> camera_points;
		for (cv::Mat point: points) {
			pcl::PointXYZRGB p = pcl::PointXYZRGB(r, g, b);
			p.x = point.at<double>(0);
			p.y = point.at<double>(1);
			p.z = point.at<double>(2);
			camera_points.push_back(p);
		}

		pcl::PolygonMesh camera_mesh;
		camera_mesh.polygons.resize(6);

		const int polygon[] = {
			0, 1, 2,
			0, 3, 1,
			0, 4, 3,
			0, 2, 4,
			3, 1, 4,
			2, 4, 1
		};

		for (int i = 0; i < 6; i++) {
			for (int j = 0; j < 3; j++) {
				camera_mesh.polygons[i].vertices.push_back(polygon[i * 3 + j]);
			}
		}

#if PCL_MAJOR_VERSION == 1 && PCL_MINOR_VERSION >= 7
		pcl::toPCLPointCloud2(camera_points, camera_mesh.cloud);
#else
		pcl::toROSMsg(camera_points, camera_mesh.cloud);
#endif

		pcl::PointXYZ line_start;
		line_start.x = position.at<double>(0);
		line_start.y = position.at<double>(1);
		line_start.z = position.at<double>(2);

		pcl::PointXYZ line_end = line_start;
		line_end.x += v_z.at<double>(0) * 3.0;
		line_end.y += v_z.at<double>(1) * 3.0;
		line_end.z += v_z.at<double>(2) * 3.0;

		std::pair<pcl::PointXYZ, pcl::PointXYZ> line = std::make_pair(line_start, line_end);

		return std::make_pair(camera_mesh, line);
	}

	void MultiCamera::show(bool show_convex_hull, bool transparent) const {
		// Visualize.
		pcl::visualization::PCLVisualizer viewer("Point Cloud");

		// Show the point cloud
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud = \
			this->point_cloud->generate_pcl_point_cloud();
		viewer.addPointCloud(point_cloud);

		int i = 0;
		for (Image* image: this->images) {
			std::ostringstream name;
			name << "Camera-" << ++i;

			std::pair<pcl::PolygonMesh, std::pair<pcl::PointXYZ, pcl::PointXYZ>> camera = \
				this->make_camera_polygon(image, 255, 0, 0, 1);

			// Add camera cube
			pcl::PolygonMesh* camera_cube = &camera.first;
			viewer.addPolygonMesh(*camera_cube, name.str());

			// Add a line to point out the viewing direction
			name << " viewing direction";
			std::pair<pcl::PointXYZ, pcl::PointXYZ> camera_line = camera.second;
			viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(
				camera_line.first, camera_line.second, 1.0, 1.0, 1.0, name.str());
		}

		// Add the convex hull
		if (show_convex_hull) {
			const pcl::PolygonMesh* convex_hull = this->point_cloud->get_convex_hull_mesh();

			if (transparent)
				viewer.addPolylineFromPolygonMesh(*convex_hull, "Convex Hull");
			else
				viewer.addPolygonMesh(*convex_hull, "Convex Hull");
		}

		while (!viewer.wasStopped()) {
			viewer.spinOnce();
		}
	}
}
