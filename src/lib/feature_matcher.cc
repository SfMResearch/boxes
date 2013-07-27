
#include <list>
#include <opencv2/opencv.hpp>
#include <string>
#include <tuple>
#include <vector>

#include <boxes/constants.h>
#include <boxes/feature_matcher.h>
#include <boxes/image.h>
#include <boxes/structs.h>

namespace Boxes {
	/*
	 * Contructor.
	 */
	BoxesFeatureMatcher::BoxesFeatureMatcher(BoxesImage* image1, BoxesImage* image2) {
		this->image1 = image1;
		this->image2 = image2;

		// Cache keypoint pointers.
		this->keypoints1 = this->image1->get_keypoints();
		this->keypoints2 = this->image2->get_keypoints();

		// Match the two given images.
		//this->match();

		// Calculate optical flow.
		this->optical_flow();

		this->draw_matches("result.jpg");

		// Calculate the fundamental matrix.
		this->fundamental_matrix = this->calculate_fundamental_matrix();

		// Calculate the essential matrix.
		this->essential_matrix = this->calculate_essential_matrix();

		// Calculate all possible camera matrices.
		std::vector<cv::Matx34d> camera_matrices = this->calculate_possible_camera_matrices();

		// Find the best camera matrix.
		this->best_camera_matrix = this->find_best_camera_matrix(&camera_matrices);
		assert(this->best_camera_matrix);

		std::cout << *best_camera_matrix << std::endl;
	}

	void BoxesFeatureMatcher::optical_flow() {
		// Remove any stale matches that might be in here.
		this->matches.clear();

		std::vector<cv::Point2f> points1;
		for (std::vector<cv::KeyPoint>::const_iterator i = this->keypoints1->begin(); i != this->keypoints1->end(); ++i) {
			points1.push_back(i->pt);
		}

		std::vector<cv::Point2f> points2;
		for (std::vector<cv::KeyPoint>::const_iterator i = this->keypoints2->begin(); i != this->keypoints2->end(); ++i) {
			points2.push_back(i->pt);
		}

		// Convert images to greyscale.
		cv::Mat greyscale1 = this->image1->get_greyscale_mat();
		cv::Mat greyscale2 = this->image2->get_greyscale_mat();

		std::vector<uchar> vstatus;
		std::vector<float> verror;

		// Calculate the optical flow field, i.e. how each point1 moved across the two images
		cv::calcOpticalFlowPyrLK(greyscale1, greyscale2, points1, points2, vstatus, verror);

		// First, filter out the points with high error.
		std::vector<cv::Point2f> good_points1;
		std::vector<int> points1_index;

		for (unsigned int i = 0; i < vstatus.size(); i++) {
			// Save good points with a low error rate.
			if (vstatus[i] && verror[i] < OF_MAX_VERROR) {
				// Keep the original index of the point in the optical flow array,
				// for future use
				points1_index.push_back(i);

				// Keep the feature point itself
				good_points1.push_back(points1[i]);
			}
		}

		/* Look around each optical flow point in the first image for any features
		 * that were detected in its area and make a match.
		 */
		std::vector<std::vector<cv::DMatch>> nearest_neighbours;

		// Format appropriate data structures.
		cv::Mat good_points1_flat = cv::Mat(good_points1).reshape(1, good_points1.size());
		cv::Mat points2_flat = cv::Mat(points2).reshape(1, points2.size());

		cv::BFMatcher matcher = cv::BFMatcher(CV_L2);
		matcher.radiusMatch(good_points1_flat, points2_flat, nearest_neighbours, OF_RADIUS_MATCH);

		/* Check if the found neighbours are unique
		 * (throw away neighbours that are too close together, as they may be confusing)
		 */
		std::set<int> found_in_points2;

		for (unsigned int i = 0; i < nearest_neighbours.size(); i++) {
			cv::DMatch match;

			unsigned int size = nearest_neighbours[i].size();

			if (size >= 1) {
				match = nearest_neighbours[i][0];
			} else {
				continue;
			}

			/* If there are two neighbours, we check how close they are.
			 * If they aren't too close, we to each other, we go with neither
			 * of them because we cannot tell which one is better. */
			if (size > 1) {
				double ratio = match.distance / nearest_neighbours[i][1].distance;

				if (ratio >= OF_NEIGHBOUR_RATIO) {
					// too close.
					continue;
				}
			}

			/* Check if the match point is already in the list.
			 * If not, add it. */
			if (found_in_points2.find(match.trainIdx) == found_in_points2.end()) {
				match.queryIdx = points1_index[match.queryIdx];
				this->matches.push_back(match);

				found_in_points2.insert(match.trainIdx);
			}
		}

		this->search_match_points();

		assert(this->matches.size() <= this->keypoints1->size());
		assert(this->matches.size() <= this->keypoints2->size());
	}

	void BoxesFeatureMatcher::match() {
		const cv::Mat* descriptors1 = this->image1->get_descriptors();
		const cv::Mat* descriptors2 = this->image2->get_descriptors();

		this->match(descriptors1, descriptors2);
	}

	void BoxesFeatureMatcher::match(const cv::Mat* descriptors1, const cv::Mat* descriptors2, int norm_type) {
		// Remove any stale matches that might be in here.
		this->matches.clear();

		// Create matcher
		cv::BFMatcher matcher = cv::BFMatcher(norm_type);

		// Match.
		std::vector<std::vector<cv::DMatch>> matches;
		matcher.knnMatch(*descriptors1, *descriptors2, matches, 2);

		for (std::vector<std::vector<cv::DMatch>>::iterator m = matches.begin(); m != matches.end(); ++m) {
			if (m->size() >= 2) {
				cv::DMatch* match1 = &m->at(0);
				cv::DMatch* match2 = &m->at(1);

				if (match1->distance > match2->distance * 0.8)
					continue;
			} else if (m->size() != 1) {
				continue;
			}

			this->matches.push_back(m->at(0));
		}

		this->search_match_points();
	}

	void BoxesFeatureMatcher::search_match_points() {
		assert(this->matches.size() > 0);

		// Discard any data.
		this->match_points1.clear();
		this->match_points2.clear();

		for (std::vector<cv::DMatch>::iterator m = this->matches.begin() ; m != this->matches.end(); ++m) {
			cv::DMatch match = *m;

			cv::KeyPoint keypoint1 = (*this->keypoints1)[match.queryIdx];
			cv::KeyPoint keypoint2 = (*this->keypoints2)[match.trainIdx];

			this->match_points1.push_back(keypoint1.pt);
			this->match_points2.push_back(keypoint2.pt);
		}
	}

	void BoxesFeatureMatcher::draw_matches(const std::string filename) {
		cv::Mat img_matches;

		const cv::Mat* image1 = this->image1->get_mat();
		const cv::Mat* image2 = this->image2->get_mat();

		cv::drawMatches(*image1, *this->keypoints1, *image2, *this->keypoints2, this->matches, img_matches);

		BoxesImage image = BoxesImage(img_matches);
		image.write(filename);
	}

	cv::Mat BoxesFeatureMatcher::calculate_fundamental_matrix() {
		return cv::findFundamentalMat(this->match_points1, this->match_points2, cv::FM_RANSAC, 0.1, 0.99);
	}

	cv::Mat BoxesFeatureMatcher::calculate_essential_matrix() {
		cv::Mat camera_matrix = this->image1->guess_camera_matrix();

		return camera_matrix.t() * this->fundamental_matrix * camera_matrix;
	}

	std::vector<cv::Matx34d> BoxesFeatureMatcher::calculate_possible_camera_matrices() {
		// Perform SVD of the matrix.
		cv::SVD svd = cv::SVD(this->essential_matrix, cv::SVD::MODIFY_A|cv::SVD::FULL_UV);

		// Check if first and second singular values are the same (as they should be).
		double singular_values_ratio = fabsf(svd.w.at<double>(0) / svd.w.at<double>(1));
		if (singular_values_ratio > 1.0)
			singular_values_ratio = 1.0 / singular_values_ratio;

		if (singular_values_ratio < 0.7) {
			// XXX should throw an exception
			std::cerr << "singular values are too far apart" << std::endl;
		}

		const cv::Matx33d W(0, -1, 0, 1, 0, 0, 0, 0, 1);
		const cv::Matx33d Wt(0, 1, 0, -1, 0, 0, 0, 0, 1);

		cv::Mat_<double> rotation1 = svd.u * cv::Mat(W) * svd.vt;
		cv::Mat_<double> rotation2 = svd.u * cv::Mat(Wt) * svd.vt;

		cv::Mat_<double> translation1 = svd.u.col(2);
		cv::Mat_<double> translation2 = -translation1;

		std::vector<cv::Matx34d> matrices;

		cv::Mat* rotation = &rotation1;
		cv::Mat* translation = &translation1;

		cv::Matx34d matrix;
		for (unsigned int i = 0; i < 2; i++) {
			translation = &translation1;

			for (unsigned int j = 0; j < 2; j++) {
				matrix = cv::Matx34d(
					rotation->at<double>(0, 0), rotation->at<double>(0, 1), rotation->at<double>(0, 2), translation->at<double>(0),
					rotation->at<double>(1, 0), rotation->at<double>(1, 1), rotation->at<double>(1, 2), translation->at<double>(1),
					rotation->at<double>(2, 0), rotation->at<double>(2, 1), rotation->at<double>(2, 2), translation->at<double>(2)
				);
				matrices.push_back(matrix);

				translation = &translation2;
			}

			rotation = &rotation2;
		}

		return matrices;
	}

	cv::Matx34d* BoxesFeatureMatcher::find_best_camera_matrix(std::vector<cv::Matx34d>* camera_matrices) {
		cv::Matx34d P0 = cv::Matx34d::eye();

		std::vector<CloudPoint> point_cloud;
		std::vector<cv::KeyPoint> corresponding_image_points;

		for (std::vector<cv::Matx34d>::iterator i = (*camera_matrices).begin(); i != (*camera_matrices).end(); ++i) {
			cv::Matx34d p = *i;

			// Check for coherency of the rotation matrix.
			cv::Mat rotation = cv::Mat(p).colRange(0, 3);
			double determinant = fabsf(cv::determinant(rotation)) - 1.0;
			if (determinant > 1e-07)
				continue;

			// Reset point cloud.
			point_cloud.clear();

			double reprojection_error = this->triangulate_points(&P0, &p, &point_cloud, &corresponding_image_points);

			// Skip all matrices with too high projection error.
			if (reprojection_error > REPROJECTION_ERROR_MAX)
				continue;

			// Test triangulation for a valid result.

			// Count all points that or "in front of the camera".
			unsigned int points_in_front = 0;
			for (std::vector<CloudPoint>::iterator cp = point_cloud.begin(); cp != point_cloud.end(); ++cp) {
				if (cp->pt.z > 0)
					points_in_front++;
			}
			double percentage = (double)points_in_front / (double)point_cloud.size();

			// If more than 75% of the points are in front of the camera, we
			// consider the camera matrix as a valid solution.
			if (percentage > 0.75)
				return &(*i);
		}

		return NULL;
	}

	double BoxesFeatureMatcher::triangulate_points(cv::Matx34d* p1, cv::Matx34d* p2, std::vector<CloudPoint>* point_cloud, std::vector<cv::KeyPoint>* corresponding_image_points) {
		cv::Mat c1 = this->image1->guess_camera_matrix();
		cv::Mat c1_inv = c1.inv();
		cv::Mat c2 = this->image2->guess_camera_matrix();
		cv::Mat c2_inv = c2.inv();

		cv::Mat_<double> KP1 = c1 * cv::Mat(*p1);

		std::vector<double> reproj_errors;

		assert(this->keypoints1->size() >= this->matches.size());
		assert(this->keypoints2->size() >= this->matches.size());

		for (unsigned int i = 0; i < this->matches.size(); i++) {
			const cv::KeyPoint* keypoint1 = &this->keypoints1->at(i);
			const cv::KeyPoint* keypoint2 = &this->keypoints2->at(i);

			const cv::Point2f* match_point1 = &keypoint1->pt;
			const cv::Point2f* match_point2 = &keypoint2->pt;

			cv::Point3d match_point_3d1(match_point1->x, match_point1->y, 1.0);
			cv::Point3d match_point_3d2(match_point2->x, match_point2->y, 1.0);

			cv::Mat_<double> match_point_3d1i = c1_inv * cv::Mat_<double>(match_point_3d1);
			cv::Mat_<double> match_point_3d2i = c2_inv * cv::Mat_<double>(match_point_3d2);

			match_point_3d1.x = match_point_3d1i(0);
			match_point_3d1.y = match_point_3d1i(1);
			match_point_3d1.z = match_point_3d1i(2);

			match_point_3d2.x = match_point_3d2i(0);
			match_point_3d2.y = match_point_3d2i(1);
			match_point_3d2.z = match_point_3d2i(2);

			// Triangulate one point.
			cv::Mat_<double> X = this->triangulate_one_point(&match_point_3d1, p1, &match_point_3d2, p2);

			// Reproject X.
			cv::Mat_<double> X_reproj = KP1 * X;
			cv::Point2f X_reproj_point = cv::Point2f(X_reproj(0) / X_reproj(2), X_reproj(1) / X_reproj(2));

			// Calculate the reprojection error.
			double reproj_error = cv::norm(X_reproj_point - *match_point1);
			reproj_errors.push_back(reproj_error);

			// Create CloudPoint object.
			CloudPoint cloud_point;
			cloud_point.pt = cv::Point3d(X(0), X(1), X(2));
			cloud_point.reprojection_error = reproj_error;

			point_cloud->push_back(cloud_point);
			corresponding_image_points->push_back(*keypoint1);
		}

		cv::Scalar mse = cv::mean(reproj_errors);
		return mse[0];
	}

	cv::Mat_<double> BoxesFeatureMatcher::triangulate_one_point(const cv::Point3d* p1, const cv::Matx34d* c1, const cv::Point3d* p2, const cv::Matx34d* c2) {
		cv::Matx43d A(
			p1->x * (*c1)(2,0) - (*c1)(0,0), p1->x * (*c1)(2,1) - (*c1)(0,1), p1->x * (*c1)(2,2) - (*c1)(0,2),
			p1->y * (*c1)(2,0) - (*c1)(1,0), p1->y * (*c1)(2,1) - (*c1)(1,1), p1->y * (*c1)(2,2) - (*c1)(1,2),
			p2->x * (*c2)(2,0) - (*c2)(0,0), p2->x * (*c2)(2,1) - (*c2)(0,1), p2->x * (*c2)(2,2) - (*c2)(0,2),
			p2->y * (*c2)(2,0) - (*c2)(1,0), p2->y * (*c2)(2,1) - (*c2)(1,1), p2->y * (*c2)(2,2) - (*c2)(1,2)
		);

		cv::Matx41d B(
			-(p1->x * (*c1)(2,3) - (*c1)(0,3)),
			-(p1->y * (*c1)(2,3) - (*c1)(1,3)),
			-(p2->x * (*c2)(2,3) - (*c2)(0,3)),
			-(p2->y * (*c2)(2,3) - (*c2)(1,3))
		);

		cv::Mat_<double> X;
		cv::solve(A, B, X, cv::DECOMP_SVD);

		cv::Mat_<double> Y(4,1);
		Y(0) = X(0);
		Y(1) = X(1);
		Y(2) = X(2);
		Y(3) = 1.0;

		return Y;
	}
}
