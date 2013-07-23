
#include <list>
#include <opencv2/opencv.hpp>
#include <tuple>

#include <boxes/boxes.h>

namespace Boxes {

	/*
	 * Contructor.
	 */
	Boxes::Boxes() {
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
}
