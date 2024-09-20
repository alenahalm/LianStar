#include <iostream>

#include <chrono>

#include <opencv2/opencv.hpp>

#include "Lian/Point.hpp"

#include "Lian/Lian.hpp"
#include "Lian/Detail/Geometry.hpp"
#include "Lian/Detail/Map.hpp"
#include "Lian/Detail/LianFunctions.hpp"
#include "Lian/Detail/Vector.hpp"
#include "Lian/Detail/WindEffect.hpp"

#define PATH_IMG "resources/irk.png"
#define PATH_IMG_SOURCE "resources/irk.png"


using namespace std;
using Algorithms::Graph::Lian::Lian;
using namespace Algorithms::Graph::Geometry;
using namespace Algorithms::Graph::Map;
using Algorithms::Graph::LianFunctions::Expand;
using Algorithms::WindEffects::makeShade;

int main() {

	cv::Mat rawImg = cv::imread(PATH_IMG, cv::IMREAD_COLOR);
	cv::Mat rawImgSource = cv::imread(PATH_IMG_SOURCE, cv::IMREAD_COLOR);

	double scale = 3; // pixels in 1m
	Vector direction(1, -1); // wind direction
	double speed = 0; // wind speed

	if (rawImg.empty()) {

		std::cerr << "Image not found!" << std::endl;

		return -1;
	}

	cv::Mat img;

	cv::cvtColor(rawImg, img, cv::COLOR_BGR2GRAY);

	img.setTo(255, img > 200);
	img.setTo(0, img != 255);


	
	//get value by index
	//std::cout << std::boolalpha << ((int)mImg.getMap().at<uchar>(cv::Point(168, 305))) << std::endl;

	// for irk
	//Point start = Point(186, 174);
	//Point goal = Point(915, 478);
	Point start = Point(213, 305);
	//Point goal = Point(1287, 689);
	Point goal = Point(1334, 554);


	// for syk
	//Point start = Point(68, 634);
	//Point goal = Point(991, 37);

	// for map
	//Point start = Point(130, 353);
	//Point goal = Point(354, 155);

	// for simple
	 /*Point start = Point(500, 420);
	 Point goal = Point(1030, 250);*/

	Point point = Point(100, 100);

	// --- testing ---

	StagePoint sP(start, Point(0, 0), 0.0, 0.0, 0.0);
	std::vector<StagePoint> close;
	std::map<Point, StagePoint> mapPath;

	Vector wind = makeWindVector(scale, speed, direction);

	img = makeShade(img, wind, scale);
	Map mImg(img);
	cv::cvtColor(rawImgSource, rawImgSource, cv::COLOR_BGR2GRAY);
	rawImgSource = makeShade(rawImgSource, wind, scale);
	cv::cvtColor(rawImgSource, rawImgSource, cv::COLOR_GRAY2BGR);
	Map mImgSource(rawImgSource);


	
	auto timer = std::chrono::steady_clock::now();
	auto resPath = Lian(start, goal, mImg, mImgSource, 10, 25, wind, scale);

	for (auto&& point : resPath) {
		cv::circle(mImg.getMap(), cv::Point(point.x, point.y), 3, cv::Scalar(100, 100, 100), -1);
	}

	std::cout << "Time code -> " << std::chrono::duration <double, std::milli>(std::chrono::steady_clock::now() - timer).count() << std::endl;
	
	// --- end testing----
	
	imshow("Display window", img);
	int k = cv::waitKey(0);

	return 0;
}