
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
	class FeatureMatcher {
		public:
			FeatureMatcher(Image* image1, Image* image2);

			CameraMatrix* run();

			const CameraMatrix* best_camera_matrix;

			void draw_matches(const std::string filename);

		protected:
			Image* image1;
			Image* image2;

			const std::vector<cv::KeyPoint>* keypoints1;
			const std::vector<cv::KeyPoint>* keypoints2;

			void match();
			void _match(const cv::Mat* descriptors1, const cv::Mat* descriptors2, const std::vector<MatchPoint>* match_points = NULL, int match_type = MATCH_TYPE_NORMAL, int norm_type = cv::NORM_L2);
			std::vector<cv::DMatch> matches;

			cv::Mat calculate_essential_matrix(cv::Mat* fundamental_matrix);
			cv::Mat calculate_fundamental_matrix();	

			CameraMatrix* find_best_camera_matrix(std::vector<CameraMatrix*>* camera_matrices);
			std::vector<CameraMatrix*> calculate_possible_camera_matrices(cv::Mat* essential_matrix);

			double triangulate_points(cv::Matx34d* p0, cv::Matx34d* p1, PointCloud* point_cloud);
			cv::Mat_<double> triangulate_one_point(const cv::Point3d* p1, const cv::Matx34d* c1, const cv::Point3d* p2, const cv::Matx34d* c2);
			cv::Mat_<double> _triangulate_one_point(const cv::Point3d* p1, const cv::Matx34d* c1, const cv::Point3d* p2, const cv::Matx34d* c2, double weight1 = 1.0, double weight2 = 1.0);

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr generate_pcl_point_cloud(const std::vector<CloudPoint> point_cloud);
	};
};

#endif
