
#include <list>
#include <opencv2/opencv.hpp>
#include <string>
#include <tuple>
#include <vector>

#include <boxes/boxes.h>
#include <boxes/config.h>
#include <boxes/feature_matcher.h>
#include <boxes/feature_matcher_optical_flow.h>
#include <boxes/image.h>

namespace Boxes {

	/*
	 * Contructor.
	 */
	Boxes::Boxes() {
		this->config = new Config();
	}

	Boxes::~Boxes() {
		delete this->config;
	}

	unsigned int Boxes::img_read(const std::string filename) {
		Image *image = new Image((Boxes *)this, filename);
		this->images.push_back(image);

		return this->img_size() - 1;
	}

	Image* Boxes::img_get(unsigned int index) {
		if (this->img_size() <= index)
			return NULL;

		return this->images[index];
	}

	unsigned int Boxes::img_size() const {
		return this->images.size();
	}

	void Boxes::set_algorithms(const std::string algorithms) {
		std::size_t pos = algorithms.find_last_of("-");

		if (pos != std::string::npos) {
			std::string detector = algorithms.substr(0, pos);
			this->config->set("FEATURE_DETECTOR", detector);

			std::string extractor = algorithms.substr(pos + 1, algorithms.size());
			this->config->set("FEATURE_DETECTOR_EXTRACTOR", extractor);
		} else {
			this->config->set("FEATURE_DETECTOR", algorithms);
			this->config->set("FEATURE_DETECTOR_EXTRACTOR", algorithms);
		}
	}

	/*
	 *
	 */
	std::string Boxes::version_string() const {
		std::string str;
		str += PACKAGE_NAME;
		str += " ";
		str += VERSION;
		str += " (";
		str += __DATE__;
		str += " ";
		str += __TIME__;
		str += ") ";

		// OpenCV
		str += "OpenCV ";
		str += CV_VERSION;

		// List active optimizations.
		std::list<std::tuple<std::string, int>> hardware_support;
		hardware_support.push_back(std::tuple<std::string, int>("MMX",      CV_CPU_MMX));
		hardware_support.push_back(std::tuple<std::string, int>("SSE",      CV_CPU_SSE));
		hardware_support.push_back(std::tuple<std::string, int>("SSE2",     CV_CPU_SSE2));
		hardware_support.push_back(std::tuple<std::string, int>("SSE3",     CV_CPU_SSE3));
		hardware_support.push_back(std::tuple<std::string, int>("SSSE3",    CV_CPU_SSSE3));
		hardware_support.push_back(std::tuple<std::string, int>("SSE4.1",   CV_CPU_SSE4_1));
		hardware_support.push_back(std::tuple<std::string, int>("SSE4.2",   CV_CPU_SSE4_2));
		hardware_support.push_back(std::tuple<std::string, int>("POPCOUNT", CV_CPU_POPCNT));
		hardware_support.push_back(std::tuple<std::string, int>("AVX",      CV_CPU_AVX));

		str += " (HW Support:";
		for (std::list<std::tuple<std::string, int>>::const_iterator i = hardware_support.begin(); i != hardware_support.end(); ++i) {
			if (cv::checkHardwareSupport(std::get<1>(*i))) {
				str += " ";
				str += std::get<0>(*i);
			}
		}
		str += ")";

		return str;
	}

	// Iterator
	std::vector<Image*>::iterator Boxes::begin() {
		return this->images.begin();
	}

	std::vector<Image*>::const_iterator Boxes::begin() const {
		return this->images.begin();
	}

	std::vector<Image*>::iterator Boxes::end() {
		return this->images.end();
	}

	std::vector<Image*>::const_iterator Boxes::end() const {
		return this->images.end();
	}

	std::vector<std::pair<Image*, Image*>> Boxes::make_pairs() const {
		std::vector<std::pair<Image*, Image*>> ret;

		Image* last_image = NULL;

		for (std::vector<Image*>::const_iterator i = this->begin(); i != this->end(); i++) {
			Image* image = *i;

			if (last_image == NULL) {
				last_image = image;
				continue;
			}

			ret.push_back(std::make_pair(last_image, image));
			last_image = image;
		}

		return ret;
	}
}
