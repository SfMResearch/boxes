
#ifndef BOXES_IMAGE_H
#define BOXES_IMAGE_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include <boxes/constants.h>

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
			cv::Mat* get_descriptors(std::vector<cv::KeyPoint>* keypoints,
				const std::string detector_type = DEFAULT_FEATURE_DETECTOR_EXTRACTOR) const;

			// keypoints
			std::vector<cv::KeyPoint>* get_keypoints(const std::string detector_type = DEFAULT_FEATURE_DETECTOR) const;

			// distance
			void set_distance(unsigned int distance);

			// camera matrix
			cv::Mat guess_camera_matrix() const;

			// disparity map
			Image* get_disparity_map(Image *other_img);

		private:
			cv::Mat mat;
			void decode_jfif_data(std::string filename);

			// distance
			unsigned int distance = 0;
	};
};

#endif
