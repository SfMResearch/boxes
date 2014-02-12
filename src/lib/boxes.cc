/***
	This file is part of the boxes library.

	Copyright (C) 2013-2014  Christian Bodenstein, Michael Tremer

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Lesser General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU Lesser General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
***/

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
#include <boxes/util.h>

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

	unsigned int Boxes::img_read(const std::string filename, const std::string resolution) {
		int width = -1;
		int height = -1;

		if(resolution != "")
		{
			std::pair<std::string, std::string> res = split_once(resolution, "x");
			std::stringstream(res.first) >> width;
			if (!res.second.empty())
				std::stringstream(res.second) >> height;
		}
		Image *image = new Image((Boxes *)this, filename, width, height);
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
		str += "lib" PACKAGE_NAME;
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
