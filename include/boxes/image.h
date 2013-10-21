
#ifndef BOXES_IMAGE_H
#define BOXES_IMAGE_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include <boxes/forward_declarations.h>
#include <boxes/constants.h>
#include <boxes/point_cloud.h>

#include <moges/Types.h>
#include <moges/NURBS/Curve.h>

namespace Boxes {
	class Image {
		public:
			Image();
			Image(const std::string filename);
			Image(cv::Mat mat);
			~Image();

			// Basic methods.
			void show();
			void write(const std::string filename);
			cv::Size size() const;

			// mat
			const cv::Mat* get_mat() const;
			const cv::Mat* get_mat(int code) const;
			const cv::Mat* get_greyscale_mat() const;

			// descriptors
			cv::Mat* get_descriptors(std::vector<cv::KeyPoint>* keypoints,
				const std::string detector_type = DEFAULT_FEATURE_DETECTOR_EXTRACTOR) const;

			// keypoints
			std::vector<cv::KeyPoint>* get_keypoints(const std::string detector_type = DEFAULT_FEATURE_DETECTOR) const;

			// (good) features
			std::vector<cv::Point2f> get_good_features_to_track(int max_corners = 1500, double quality_level = 0.05,
				double min_distance = 2.0);

			// distance
			void set_distance(unsigned int distance);

			// camera matrix
			cv::Mat guess_camera_matrix() const;

			// disparity map
			Image* get_disparity_map(Image *other_img);

			// curve
			bool has_curve() const;
			std::vector<MoGES::IntPoint> discretize_curve() const;
			void draw_curve(double colour = 0xffffff);
			PointCloud cut_out_curve(PointCloud* cloud) const;

		private:
			cv::Mat mat;
			void decode_jfif_data(std::string filename);

			// curve
			MoGES::NURBS::Curve* curve = NULL;
			MoGES::NURBS::Curve* read_curve(const std::string filename) const;
			std::string find_curve_file(const std::string filename) const;

			// distance
			unsigned int distance = 0;
	};
};

#endif
