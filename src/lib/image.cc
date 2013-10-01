
#include <boost/regex.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include <boxes/constants.h>
#include <boxes/image.h>

namespace Boxes {
	Image::Image() {
	}

	Image::Image(const std::string filename) {
		this->mat = cv::imread(filename);

		// Read available JFIF data.
		this->decode_jfif_data(filename);

		assert(!this->mat.empty());
	}

	Image::Image(cv::Mat mat) {
		this->mat = mat;
	}

	Image::~Image() {
	}

	void Image::decode_jfif_data(std::string filename) {
		unsigned char header[2048];
		unsigned int pos = 0;

		std::ifstream file(filename, std::ios::in|std::ios::binary);
		if (file.is_open()) {
			file.read((char *)header, sizeof(header));
			file.close();

			// Check for magic number match.
			if(header[pos] == 0xFF && header[pos+1] == 0xD8 && header[pos+2] == 0xFF && header[pos+3] == 0xE0) {
				pos += 4;

				// Check for a valid JFIF header (JFIF\0).
				if (header[pos+2] == 'J' && header[pos+3] == 'F' && header[pos+4] == 'I' && header[pos+5] == 'F' && header[pos+6] == 0x00) {
					// Retrieve block by block...
					unsigned int block_length = (header[pos] << 8) + header[pos+1];
					while (pos < sizeof(header)) {
						pos += block_length;

						// Check if we jumped past the end of the header.
						if (pos >= sizeof(header)) {
							break;
						}

						// Handle comment block.
						if (header[pos+1] == 0xFE) {
							unsigned int comment_pos = 0;
							unsigned int comment_length = (header[pos+2] << 8) + header[pos+3] - 2;

							// Copy comment to string.
							std::string comment;
							while (comment_pos < comment_length) {
								comment += header[pos+comment_pos+4];
								comment_pos++;
							}

							// Go (for) the distance.
							/* Using boost::regex here, becase std::regex::regex_search is broken since GCC 4.7 */
							boost::regex re("Distance=\"(\\d+)\"", boost::regex_constants::extended);

							boost::smatch matches;
							if (boost::regex_search(comment, matches, re)) {
								unsigned int distance = 0;
								std::stringstream (matches[1]) >> distance;

								this->set_distance(distance);
							}
						}

						// Go to the next block.
						block_length = (header[pos+2] << 8) + header[pos+3] + 2;
					}
				}
			}
		}
	}

	void Image::show() {
		cv::namedWindow("Image", CV_WINDOW_AUTOSIZE);
		cv::imshow("Image", this->mat);
	}

	void Image::write(const std::string filename) {
		cv::imwrite(filename, this->mat);
	}

	const cv::Mat* Image::get_mat() const {
		return &this->mat;
	}

	cv::Size Image::size() const {
		return this->mat.size();
	}

	std::vector<cv::KeyPoint>* Image::get_keypoints(const std::string detector_type) const {
		std::vector<cv::KeyPoint>* output = new std::vector<cv::KeyPoint>();

		cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create(detector_type);
		detector->detect(this->mat, *output);

		return output;
	}

	cv::Mat* Image::get_descriptors(std::vector<cv::KeyPoint>* keypoints, const std::string detector_type) const {
		cv::Mat* descriptors = new cv::Mat();

		// Extract descriptors.
		cv::Ptr<cv::DescriptorExtractor> extractor = cv::DescriptorExtractor::create(detector_type);
		extractor->compute(this->mat, *keypoints, *descriptors);

		return descriptors;
	}

	cv::Mat Image::get_greyscale_mat() {
		if (this->mat.channels() == 1) {
			return this->mat;
		}

		cv::Mat greyscale;
		cv::cvtColor(this->mat, greyscale, CV_RGB2GRAY);

		return greyscale;
	}

	void Image::set_distance(unsigned int distance) {
		this->distance = distance;
	}

	cv::Mat Image::guess_camera_matrix() const {
		cv::Size image_size = this->size();

		cv::Mat camera_matrix = cv::Mat::zeros(3, 3, CV_64F);
		camera_matrix.at<double>(0, 0) = image_size.width;
		camera_matrix.at<double>(1, 1) = image_size.height;
		camera_matrix.at<double>(2, 2) = 1.0;

		// The center of the image.
		camera_matrix.at<double>(0, 2) = image_size.width / 2;
		camera_matrix.at<double>(1, 2) = image_size.height / 2;

		return camera_matrix;
	}

	Image* Image::get_disparity_map(Image *other_img) {
		cv::Mat disparity_map;

		cv::StereoBM stereoBM;
		stereoBM(this->get_greyscale_mat(), other_img->get_greyscale_mat(), disparity_map);

		return new Image(disparity_map);
	}
};
