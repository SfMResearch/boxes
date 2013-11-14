
#include <boost/regex.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include <boxes/constants.h>
#include <boxes/image.h>

#include <moges/Types.h>
#include <moges/NURBS/Curve.h>

namespace Boxes {
	Image::Image() {
	}

	Image::Image(const std::string filename) {
		this->mat = cv::imread(filename);
		assert(!this->mat.empty());

		// Read available JFIF data.
		this->decode_jfif_data(filename);

		// Try to find a curve and read it.
		std::string filename_curve = this->find_curve_file(filename);
		if (!filename_curve.empty()) {
			this->curve = this->read_curve(filename_curve);
		}
	}

	Image::Image(cv::Mat mat) {
		this->mat = mat;
	}

	Image::~Image() {
		if (this->curve)
			delete this->curve;

		// Delete all cached keypoints.
		for (std::map<const std::string, std::vector<cv::KeyPoint>*>::iterator i = this->keypoints.begin();
				i != this->keypoints.end(); i++) {
			delete i->second;
		}
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

	const cv::Mat* Image::get_mat(int code) const {
		if (this->mat.type() == code) {
			return &this->mat;
		}

		cv::Mat* new_mat = new cv::Mat();
		cv::cvtColor(this->mat, *new_mat, code);

		return new_mat;
	}

	const cv::Mat* Image::get_greyscale_mat() const {
		return this->get_mat(CV_RGB2GRAY);
	}

	cv::Size Image::size() const {
		return this->mat.size();
	}

	std::vector<cv::KeyPoint>* Image::get_keypoints(const std::string detector_type) {
		std::vector<cv::KeyPoint>* keypoints;

		#pragma omp critical
		{
			keypoints = this->keypoints[detector_type];

			if (!keypoints) {
				keypoints = this->compute_keypoints(detector_type);
				this->keypoints[detector_type] = keypoints;
			}
		}

		return keypoints;
	}

	std::vector<cv::KeyPoint>* Image::compute_keypoints(const std::string detector_type) const {
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

	std::vector<cv::Point2f> Image::get_good_features_to_track(int max_corners, double quality_level, double min_distance) {
		const cv::Mat* mat = this->get_greyscale_mat();

		std::vector<cv::Point2f> corners = std::vector<cv::Point2f>();

		// Find all good features to track.
		cv::goodFeaturesToTrack(*mat, corners, max_corners, quality_level, min_distance);

		// Increase the precision of the result.
		cv::cornerSubPix(*mat, corners, cv::Size(15, 15), cv::Size(-1, -1),
			cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));

		return corners;
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
		stereoBM(*this->get_greyscale_mat(), *other_img->get_greyscale_mat(), disparity_map);

		return new Image(disparity_map);
	}

	bool Image::has_curve() const {
		return (this->curve != NULL);
	}

	MoGES::NURBS::Curve* Image::read_curve(const std::string filename) const {
		MoGES::NURBS::Curve* curve = new MoGES::NURBS::Curve();
		curve->read(filename);

		return curve;
	}

	std::string Image::find_curve_file(const std::string filename) const {
		std::string result;

		// Build a vector of candidate filenames.
		std::vector<std::string> filenames;
		filenames.push_back(filename + "." + NURBS_CURVE_EXTENSION);

		// Replace extension by nurbs file extension.
		std::size_t pos = filename.find_last_of(".");
		if (pos != std::string::npos) {
			std::string basename = std::string(filename.substr(0, pos));
			filenames.push_back(basename + "." + NURBS_CURVE_EXTENSION);
		}

		// Check for all filenames in the vector if they exist
		for (std::vector<std::string>::const_iterator i = filenames.begin(); i != filenames.end(); i++) {
			// Check if the file can be opened for reading.
			std::ifstream file(*i, std::ios::in);
			if (!file.is_open())
				continue;

			// File exists.
			file.close();

			result.assign(*i);
			break;
		}

		return result;
	}

	std::vector<cv::Point2f> Image::discretize_curve() const {
		std::vector<cv::Point2f> result;

		if (this->has_curve()) {
			MoGES::NURBS::DiscreteCurvePtr discrete_curve = this->curve->discretize();

			for (MoGES::NURBS::DiscreteCurve::iterator i = discrete_curve->begin(); i != discrete_curve->end(); i++) {
				cv::Point2f point = cv::Point2f(i->second[0], i->second[1]);
				result.push_back(point);
			}
		}

		return result;
	}

	void Image::draw_curve() {
		std::vector<cv::Point2f> discrete_curve = this->discretize_curve();

		for (std::vector<cv::Point2f>::iterator i = discrete_curve.begin(); i != discrete_curve.end(); i++) {
			if (i->x < 0 || i->x > this->mat.cols)
				continue;

			if (i->y < 0 || i->y > this->mat.rows)
				continue;

			this->mat.at<cv::Vec3b>(i->y, i->x)[1] = 255;
		}
	}

	void Image::cut_out_curve(PointCloud* point_cloud) const {
		if (this->has_curve()) {
			std::vector<cv::Point2f> discrete_curve = this->discretize_curve();

			/* Go through the existing point cloud point by point and
			 * check if the point is within or on the contour of the curve. */
			for (std::vector<CloudPoint>::const_iterator i = point_cloud->begin(); i != point_cloud->end(); i++) {
				double distance = cv::pointPolygonTest(discrete_curve, i->keypoint.pt, false);

				if (!GREATER_OR_EQUAL_TO_ZERO(distance)) {
					point_cloud->remove_point(&(*i));
				}
			}
		}
	}
};
