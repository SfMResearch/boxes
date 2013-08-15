
#ifndef BOXES_IMAGE_H
#define BOXES_IMAGE_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

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
			cv::Mat get_greyscale_mat();

			// descriptors
			cv::Mat* get_descriptors();

			// keypoints
			const std::vector<cv::KeyPoint>* get_keypoints();

			// distance
			void set_distance(unsigned int distance);

			// camera matrix
			cv::Mat guess_camera_matrix() const;

		private:
			cv::Mat mat;
			void decode_jfif_data(std::string filename);

			// descriptors
			cv::Mat* descriptors = NULL;
			cv::Mat* calc_descriptors();

			// keypoints
			std::vector<cv::KeyPoint>* keypoints = NULL;
			std::vector<cv::KeyPoint>* calc_keypoints();

			// distance
			unsigned int distance = 0;
	};
};

#endif
