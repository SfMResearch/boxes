
#include <vector>

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
	}

	std::pair<Image*, Image*> MultiCamera::get_image_pair(unsigned int pair_index) const {
		if (pair_index < this->image_pairs.size())
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
		#pragma omp barrier

		CameraMatrix* camera_matrix = NULL;

		for (FeatureMatcher* matcher: this->feature_matchers) {
			Image* image1 = matcher->image1;
			Image* image2 = matcher->image2;

			// For the first match, find the best camera matrix of the image
			// pair and initialize the point cloud.
			if (!camera_matrix) {
				camera_matrix = matcher->calculate_camera_matrix();
				std::cout << camera_matrix->matrix << std::endl;


				/* Strip all points from the point cloud, if they are not within the
				 * NURBS curve (if that one is available).
				 */
				if (image2->has_curve()) {
					image2->cut_out_curve(&camera_matrix->point_cloud);
				}

				// Merge all points from the best camera matrix's point cloud.
				this->point_cloud->merge(&camera_matrix->point_cloud);

				continue;
			}

			std::vector<cv::KeyPoint>* keypoints2 = image2->get_keypoints();

			std::vector<cv::Point3f> local_point_cloud;
			std::vector<cv::Point2f> image_points;
			std::vector<int> point_cloud_status(this->point_cloud->size(), 0);

			for (std::vector<CloudPoint>::iterator i = this->point_cloud->begin(); i != this->point_cloud->end(); i++) {
				int keypoint2_index = matcher->find_corresponding_keypoint(i->keypoint_index);

				if (keypoint2_index >= 0) {
					cv::KeyPoint keypoint2 = (*keypoints2)[keypoint2_index];

					local_point_cloud.push_back(i->pt);
					image_points.push_back(keypoint2.pt);
				}

				i->keypoint_index = keypoint2_index;
			}

			cv::Mat_<double> rvec;
			cv::Mat_<double> translation;
			std::vector<double> distortion_coeff;
			std::vector<int> inliers;

			cv::solvePnPRansac(local_point_cloud, image_points, image1->guess_camera_matrix(),
				distortion_coeff, rvec, translation, false, 100, 8.0, 100, inliers);

			cv::Mat_<double> rotation;
			cv::Rodrigues(rvec, rotation);

			// Compose combined rotation and translation matrix.
			cv::Matx34d matrix = merge_rotation_and_translation_matrix(&rotation, &translation);

			matcher->triangulate_points(&camera_matrix->matrix, &matrix, this->point_cloud);
			camera_matrix->matrix = matrix;

			std::cout << matrix << std::endl;
		}

		std::cout << camera_matrix->matrix << std::endl;

		std::cout << this->point_cloud->size() << std::endl;

		this->point_cloud->write("point_cloud.pcd");

		if (camera_matrix)
			delete camera_matrix;
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

	void MultiCamera::write_disparity_map(unsigned int pair_index, std::string filename) const {
		std::pair<Image*, Image*> pair = this->get_image_pair(pair_index);

		Boxes::Image* disparity_map = pair.first->get_disparity_map(pair.second);
		if (disparity_map) {
			disparity_map->write(filename);
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
}
