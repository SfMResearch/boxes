/***
	This file is part of the boxes library.

	Copyright (C) 2013-2014  Christian Bodenstein, Michael Tremer

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Lesser General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU Lesser General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
***/

#include <list>
#include <opencv2/opencv.hpp>
#include <string>
#include <tuple>
#include <vector>

#include <boxes/camera_matrix.h>
#include <boxes/constants.h>
#include <boxes/converters.h>
#include <boxes/feature_matcher.h>
#include <boxes/image.h>
#include <boxes/structs.h>

namespace Boxes {
	/*
	 * Contructor.
	 */
	FeatureMatcher::FeatureMatcher(Boxes* boxes, Image* image1, Image* image2) {
		this->boxes = boxes;

		this->image1 = image1;
		this->image2 = image2;

		this->point_cloud = new PointCloud(this->boxes);
	}

	FeatureMatcher::~FeatureMatcher() {
		delete this->point_cloud;
	}

	void FeatureMatcher::match() {
		std::vector<cv::KeyPoint>* keypoints1 = this->image1->get_keypoints();
		std::vector<cv::KeyPoint>* keypoints2 = this->image2->get_keypoints();

		cv::Mat* descriptors1 = this->image1->get_descriptors(keypoints1);
		cv::Mat* descriptors2 = this->image2->get_descriptors(keypoints2);

		this->_match(descriptors1, descriptors2);

		delete descriptors1;
		delete descriptors2;
	}

	void FeatureMatcher::_match(const cv::Mat* descriptors1, const cv::Mat* descriptors2, const std::vector<MatchPoint>* match_points, int match_type, int norm_type) {
		// Remove any stale matches that might be in here.
		this->matches.clear();

		const std::vector<cv::KeyPoint>* keypoints1 = this->image1->get_keypoints();
		const std::vector<cv::KeyPoint>* keypoints2 = this->image2->get_keypoints();

		// Create matcher
		std::vector<std::vector<cv::DMatch>> nearest_neighbours;
		cv::BFMatcher matcher = cv::BFMatcher(norm_type);

		// Match.
		switch (match_type) {
			case MATCH_TYPE_NORMAL:
				matcher.knnMatch(*descriptors1, *descriptors2, nearest_neighbours, 2);
				break;

			case MATCH_TYPE_RADIUS:
				matcher.radiusMatch(*descriptors1, *descriptors2, nearest_neighbours, OF_RADIUS_MATCH);
				break;

			default:
				return;
		}

		cv::DMatch* match1;
		cv::DMatch* match2;
		for (std::vector<std::vector<cv::DMatch>>::iterator n = nearest_neighbours.begin(); n != nearest_neighbours.end(); ++n) {
			unsigned int size = n->size();

			switch (size) {
				case 0:
					continue;

#ifdef FEATURE_MATCHER_USE_SINGLE_MATCHES
				case 1:
					match1 = &n->at(0);
					break;
#endif

#ifdef FEATURE_MATCHER_USE_DOUBLE_MATCHES
				default:
					match1 = &n->at(0);
					match2 = &n->at(1);
					double match_ratio = this->boxes->config->get_double("MATCH_VALID_RATIO");
					if (match1->distance > match2->distance * match_ratio)
						continue;
					break;
#endif
			}

			MatchPoint mp;
			mp.pt1 = keypoints1->at(match1->queryIdx).pt;
			mp.pt2 = keypoints2->at(match1->trainIdx).pt;
			mp.distance = match1->distance;

			this->matches.push_back(mp);
		}

		// Prepare the fundamental matrix.
		// This will also modify the match, hence it needs to be done here.
		this->calculate_fundamental_matrix();
	}

	void FeatureMatcher::draw_matches(const std::string filename) {
		cv::Mat img_matches;

		const cv::Mat* image1 = this->image1->get_mat();
		const cv::Mat* image2 = this->image2->get_mat();

		const std::vector<cv::KeyPoint>* keypoints1 = this->image1->get_keypoints();
		const std::vector<cv::KeyPoint>* keypoints2 = this->image2->get_keypoints();

		std::vector<cv::DMatch> matches;
		for (MatchPoint mp: this->matches) {
			cv::DMatch match = match_point_to_dmatch(&mp, keypoints1, keypoints2);
			matches.push_back(match);
		}

		cv::drawMatches(*image1, *keypoints1, *image2, *keypoints2, matches, img_matches);

		Image image = Image(this->boxes, img_matches);
		image.write(filename);
	}

	const cv::Mat* FeatureMatcher::get_fundamental_matrix() const {
		return &this->fundamental_matrix;
	}

	void FeatureMatcher::calculate_fundamental_matrix() {
		std::vector<cv::Point2f> match_points1;
		std::vector<cv::Point2f> match_points2;
		for (std::vector<MatchPoint>::const_iterator i = this->matches.begin(); i != this->matches.end(); i++) {
			match_points1.push_back(i->pt1);
			match_points2.push_back(i->pt2);
		}

		double val_min = 0.0, val_max = 0.0;
		cv::minMaxIdx(match_points1, &val_min, &val_max);

		double epipolar_distance_factor = this->boxes->config->get_double("EPIPOLAR_DISTANCE_FACTOR");
		// Snavely
		double epipolar_distance = epipolar_distance_factor * val_max;

		std::vector<uchar> status(this->matches.size());

		this->fundamental_matrix = cv::findFundamentalMat(match_points1, match_points2, status,
			cv::FM_RANSAC, epipolar_distance, 0.99);

		// Sort out bad matches.
		std::vector<MatchPoint> best_matches;
		for(unsigned int i = 0; i < status.size(); i++) {
			if (status[i] == 1)
				best_matches.push_back(this->matches[i]);
		}

		this->matches = best_matches;
	}

	cv::Mat FeatureMatcher::calculate_essential_matrix() {
		cv::Mat camera = this->image1->get_camera();

		return camera.t() * this->fundamental_matrix * camera;
	}

	std::vector<CameraMatrix*> FeatureMatcher::calculate_possible_camera_matrices(const cv::Mat* essential_matrix, bool check_coherency) {
		// Perform SVD of the matrix.
		cv::SVD svd = cv::SVD(*essential_matrix, cv::SVD::FULL_UV);

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

		std::vector<CameraMatrix*> matrices;

		cv::Mat_<double>* rotation = &rotation1;
		cv::Mat_<double>* translation = &translation1;

		// Check if the rotation matrix is coherent
		if (check_coherency) {
			double determinant = cv::determinant(rotation1);

			if (IS_ZERO(determinant + 1.0)) {
				cv::Mat essential_matrix_inv = -cv::Mat(*essential_matrix);
				return this->calculate_possible_camera_matrices(&essential_matrix_inv, false);
			}
		}

		cv::Matx34d matrix;
		for (unsigned int i = 0; i < 2; i++) {
			translation = &translation1;

			for (unsigned int j = 0; j < 2; j++) {
				cv::Matx34d matrix = merge_rotation_and_translation_matrix(rotation, translation);

				CameraMatrix* camera_matrix = new CameraMatrix(this->boxes, matrix);
				matrices.push_back(camera_matrix);

				translation = &translation2;
			}

			rotation = &rotation2;
		}

		return matrices;
	}

	CameraMatrix* FeatureMatcher::calculate_camera_matrix() {
		// Calculate the essential matrix.
		cv::Mat essential_matrix = this->calculate_essential_matrix();

		// Calculate all possible camera matrices.
		std::vector<CameraMatrix*> camera_matrices = \
			this->calculate_possible_camera_matrices(&essential_matrix);

		cv::Matx34d P0 = cv::Matx34d::eye();

		CameraMatrix* best_matrix = NULL;
		for (std::vector<CameraMatrix*>::iterator i = camera_matrices.begin(); i != camera_matrices.end(); ++i) {
			CameraMatrix* camera_matrix = *i;

			// Check for coherency of the rotation matrix.
			if (!camera_matrix->rotation_is_coherent())
				continue;

			// Pick first matrix as the best one until we know better.
			if (!best_matrix)
				best_matrix = &(*camera_matrix);

			// Triangulate.
			this->triangulate_points(&P0, &(camera_matrix->matrix), camera_matrix->point_cloud);

			// Count all points that are "in front of the camera".
			if (camera_matrix->percentage_of_points_in_front_of_camera() > best_matrix->percentage_of_points_in_front_of_camera()) {
				best_matrix = &(*camera_matrix);
				continue;
			}
		}

		// Clear all camera matrices, we do not want to use.
		for (std::vector<CameraMatrix*>::iterator i = camera_matrices.begin(); i != camera_matrices.end(); ++i) {
			if (*i == best_matrix)
				continue;

			delete *i;
		}

		this->point_cloud->clear();
		this->point_cloud->merge(best_matrix->point_cloud);
		this->image2->update_camera_matrix(best_matrix);

		return best_matrix;
	}

	double FeatureMatcher::triangulate_points() {
		return this->triangulate_points(
			&this->image1->camera_matrix->matrix,
			&this->image2->camera_matrix->matrix,
			this->point_cloud
		);
	}

	double FeatureMatcher::triangulate_points(const cv::Matx34d* p1, const cv::Matx34d* p2, PointCloud* point_cloud) {
		cv::Mat c1 = this->image1->get_camera();
		cv::Mat c1_inv = c1.inv();
		cv::Mat c2 = this->image2->get_camera();
		cv::Mat c2_inv = c2.inv();

		cv::Mat_<double> KP2 = c2 * cv::Mat(*p2);

		#pragma omp parallel for
		for (unsigned int i = 0; i < this->matches.size(); i++) {
			const MatchPoint* match = &this->matches[i];

			// Create CloudPoint object.
			CloudPoint cloud_point;
			cloud_point.pt1 = match->pt1;
			cloud_point.pt2 = match->pt2;

			cv::Point3d match_point_3d1(cloud_point.pt1.x, cloud_point.pt1.y, 1.0);
			cv::Point3d match_point_3d2(cloud_point.pt2.x, cloud_point.pt2.y, 1.0);

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
			cv::Mat_<double> X_reproj = KP2 * X;
			cv::Point2f X_reproj_point = cv::Point2f(X_reproj(0) / X_reproj(2), X_reproj(1) / X_reproj(2));

			cloud_point.pt = cv::Point3d(X(0), X(1), X(2));
			cloud_point.set_colour_from_image(this->image2);

			// Calculate the reprojection error.
			cloud_point.reprojection_error = cv::norm(X_reproj_point - cloud_point.pt2);

			#pragma omp critical
			{
				point_cloud->add_point(cloud_point);
				#pragma omp flush(point_cloud)
			}
		}

		// Calculate mean reprojection error.
		std::vector<double> reproj_errors(point_cloud->size());
		const std::vector<CloudPoint>* points = point_cloud->get_points();

		for (std::vector<CloudPoint>::const_iterator i = points->begin(); i != points->end(); ++i) {
			reproj_errors.push_back(i->reprojection_error);
		}

		cv::Scalar mse = cv::mean(reproj_errors);
		this->reprojection_error = mse[0];
		return mse[0];
	}

	cv::Mat_<double> FeatureMatcher::triangulate_one_point(const cv::Point3d* p1, const cv::Matx34d* c1, const cv::Point3d* p2, const cv::Matx34d* c2) {
		cv::Mat_<double> X;

		// Initialize weights
		double weight1 = 1.0;
		double weight2 = 1.0;

		X = this->_triangulate_one_point(p1, c1, p2, c2);

		// It is suggested to run 10 iterations at most.
		for (unsigned int i = 0; i < TRIANGULATION_MAX_ITERATIONS; i++) {
			// Recalculate weights
			double weight1_ = cv::Mat_<double>(cv::Mat_<double>(*c1).row(2) * X)(0);
			double weight2_ = cv::Mat_<double>(cv::Mat_<double>(*c2).row(2) * X)(0);

			// Stop the loop, if the gained precision is smaller than epsilon.
			if ((fabsf(weight1 - weight1_) <= TRIANGULATION_EPSILON) && (fabsf(weight2 - weight2_) <= TRIANGULATION_EPSILON))
				break;

			// Otherwise use new weights and re-weight the equations.
			weight1 = weight1_;
			weight2 = weight2_;

			X = this->_triangulate_one_point(p1, c1, p2, c2, weight1, weight2);
		}

		return X;
	}

	cv::Mat_<double> FeatureMatcher::_triangulate_one_point(const cv::Point3d* p1, const cv::Matx34d* c1, const cv::Point3d* p2, const cv::Matx34d* c2, double weight1, double weight2) {
		cv::Matx43d A(
			(p1->x * (*c1)(2,0) - (*c1)(0,0)) / weight1, (p1->x * (*c1)(2,1) - (*c1)(0,1)) / weight1, (p1->x * (*c1)(2,2) - (*c1)(0,2)) / weight1,
			(p1->y * (*c1)(2,0) - (*c1)(1,0)) / weight1, (p1->y * (*c1)(2,1) - (*c1)(1,1)) / weight1, (p1->y * (*c1)(2,2) - (*c1)(1,2)) / weight1,
			(p2->x * (*c2)(2,0) - (*c2)(0,0)) / weight2, (p2->x * (*c2)(2,1) - (*c2)(0,1)) / weight2, (p2->x * (*c2)(2,2) - (*c2)(0,2)) / weight2,
			(p2->y * (*c2)(2,0) - (*c2)(1,0)) / weight2, (p2->y * (*c2)(2,1) - (*c2)(1,1)) / weight2, (p2->y * (*c2)(2,2) - (*c2)(1,2)) / weight2
		);

		cv::Matx41d B(
			-(p1->x * (*c1)(2,3) - (*c1)(0,3)) / weight1,
			-(p1->y * (*c1)(2,3) - (*c1)(1,3)) / weight1,
			-(p2->x * (*c2)(2,3) - (*c2)(0,3)) / weight2,
			-(p2->y * (*c2)(2,3) - (*c2)(1,3)) / weight2
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

	cv::Point2f* FeatureMatcher::find_corresponding_keypoint_coordinates(cv::Point2f* pt1) const {
		for (MatchPoint match: this->matches) {
			if (match.pt1 == *pt1)
				return new cv::Point2f(match.pt2);
		}

		return NULL;
	}
}
