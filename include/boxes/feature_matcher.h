
#ifndef BOXES_FEATURE_MATCHER_H
#define BOXES_FEATURE_MATCHER_H

#include <opencv2/opencv.hpp>
#include <vector>

#include <boxes/boxes.h>
#include <boxes/camera_matrix.h>
#include <boxes/constants.h>
#include <boxes/image.h>
#include <boxes/point_cloud.h>
#include <boxes/structs.h>

#define FEATURE_MATCHER_USE_SINGLE_MATCHES
#define FEATURE_MATCHER_USE_DOUBLE_MATCHES

namespace Boxes {
	class FeatureMatcher {
		public:
			FeatureMatcher(Boxes* boxes, Image* image1, Image* image2);
			virtual ~FeatureMatcher();

			Image* image1;
			Image* image2;
			PointCloud* point_cloud;

			virtual void match();
			CameraMatrix* calculate_camera_matrix();

			virtual void draw_matches(const std::string filename);

			double triangulate_points();
			double triangulate_points(const cv::Matx34d* p1, const cv::Matx34d* p2, PointCloud* point_cloud);
			double reprojection_error;

		protected:
			Boxes* boxes = NULL;

			void _match(const cv::Mat* descriptors1, const cv::Mat* descriptors2, const std::vector<MatchPoint>* match_points = NULL, int match_type = MATCH_TYPE_NORMAL, int norm_type = cv::NORM_L2);
			std::vector<MatchPoint> matches;

			// fundamental matrix
			cv::Mat fundamental_matrix;
			const cv::Mat* get_fundamental_matrix() const;
			void calculate_fundamental_matrix();

			// essential matrix
			cv::Mat calculate_essential_matrix();

			std::vector<CameraMatrix*> calculate_possible_camera_matrices(const cv::Mat* essential_matrix, bool check_coherency = true);

			cv::Mat_<double> triangulate_one_point(const cv::Point3d* p1, const cv::Matx34d* c1, const cv::Point3d* p2, const cv::Matx34d* c2);
			cv::Mat_<double> _triangulate_one_point(const cv::Point3d* p1, const cv::Matx34d* c1, const cv::Point3d* p2, const cv::Matx34d* c2, double weight1 = 1.0, double weight2 = 1.0);

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr generate_pcl_point_cloud(const std::vector<CloudPoint> point_cloud);
	};
};

#endif
