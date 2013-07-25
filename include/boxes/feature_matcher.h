
#ifndef BOXES_FEATURE_MATCHER_H
#define BOXES_FEATURE_MATCHER_H

#include <opencv2/opencv.hpp>
#include <vector>

#include <boxes/image.h>
#include <boxes/structs.h>

namespace Boxes {
	class BoxesFeatureMatcher {
		public:
			BoxesFeatureMatcher(BoxesImage* image1, BoxesImage* image2);

		private:
			BoxesImage* image1;
			BoxesImage* image2;

			const std::vector<cv::KeyPoint>* keypoints1;
			const std::vector<cv::KeyPoint>* keypoints2;

			void match();
			std::vector<cv::DMatch> matches;
			std::vector<cv::Point2f> match_points1;
			std::vector<cv::Point2f> match_points2;

			cv::Mat calculate_essential_matrix();
			cv::Mat essential_matrix;
			cv::Mat calculate_fundamental_matrix();	
			cv::Mat fundamental_matrix;

			cv::Matx34d* find_best_camera_matrix(std::vector<cv::Matx34d>* camera_matrices);
			cv::Matx34d* best_camera_matrix;
			std::vector<cv::Matx34d> calculate_possible_camera_matrices();

			double triangulate_points(cv::Matx34d* p0, cv::Matx34d* p1, std::vector<CloudPoint>* point_cloud, std::vector<cv::KeyPoint>* corresponding_image_points);
			cv::Mat_<double> triangulate_one_point(const cv::Point3d* p1, const cv::Matx34d* c1, const cv::Point3d* p2, const cv::Matx34d* c2);
	};
};

#endif
