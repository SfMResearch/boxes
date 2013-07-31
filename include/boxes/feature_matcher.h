
#ifndef BOXES_FEATURE_MATCHER_H
#define BOXES_FEATURE_MATCHER_H

#include <opencv2/opencv.hpp>
#include <vector>

#include <boxes/camera_matrix.h>
#include <boxes/constants.h>
#include <boxes/image.h>
#include <boxes/structs.h>

#define FEATURE_MATCHER_USE_SINGLE_MATCHES
#define FEATURE_MATCHER_USE_DOUBLE_MATCHES

namespace Boxes {
	class BoxesFeatureMatcher {
		public:
			BoxesFeatureMatcher(BoxesImage* image1, BoxesImage* image2);

			void draw_matches(const std::string filename);

		private:
			BoxesImage* image1;
			BoxesImage* image2;

			const std::vector<cv::KeyPoint>* keypoints1;
			const std::vector<cv::KeyPoint>* keypoints2;

			void optical_flow();
			void match();
			void match(const cv::Mat* descriptors1, const cv::Mat* descriptors2, const std::vector<MatchPoint>* match_points = NULL, int match_type = MATCH_TYPE_NORMAL, int norm_type = cv::NORM_L2);
			std::vector<cv::DMatch> matches;
			std::vector<cv::Point2f> match_points1;
			std::vector<cv::Point2f> match_points2;

			cv::Mat calculate_essential_matrix();
			cv::Mat essential_matrix;
			cv::Mat calculate_fundamental_matrix();	
			cv::Mat fundamental_matrix;

			CameraMatrix* find_best_camera_matrix(std::vector<CameraMatrix>* camera_matrices);
			CameraMatrix* best_camera_matrix;
			std::vector<CameraMatrix> calculate_possible_camera_matrices();

			double triangulate_points(cv::Matx34d* p0, cv::Matx34d* p1, std::vector<CloudPoint>* point_cloud);
			cv::Mat_<double> triangulate_one_point(const cv::Point3d* p1, const cv::Matx34d* c1, const cv::Point3d* p2, const cv::Matx34d* c2);
	};
};

#endif
