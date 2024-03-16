#include <iostream>

#include <chrono>

#include <opencv2/opencv.hpp>

#include "Lian/Point.hpp"

#include "Lian/Lian.hpp"
#include "Lian/Detail/Geometry.hpp"
#include "Lian/Detail/Map.hpp"
#include "Lian/Detail/LianFunctions.hpp"

#define PATH_IMG "resources/map1k.png"
#define PATH_IMG_SOURCE "resources/map1k.png"

using namespace std;
using Algorithms::Graph::Lian::Lian;
using namespace Algorithms::Graph::Geometry;
using namespace Algorithms::Graph::Map;
using Algorithms::Graph::LianFunctions::Expand;

int main() {

	cv::Mat rawImg = cv::imread(PATH_IMG, cv::IMREAD_COLOR);
	cv::Mat rawImgSource = cv::imread(PATH_IMG_SOURCE, cv::IMREAD_COLOR);

	if (rawImg.empty()) {

		std::cerr << "Image not found!" << std::endl;

		return -1;
	}

	cv::Mat img;

	cv::cvtColor(rawImg, img, cv::COLOR_BGR2GRAY);

	img.setTo(255, img > 200);
	img.setTo(0, img != 255);

	Map mImg(img);
	Map mImgSource(rawImgSource);
	
	//get value by index
	//std::cout << std::boolalpha << ((int)mImg.getMap().at<uchar>(cv::Point(168, 305))) << std::endl;

	// for map1k
	Point start = Point(215, 300);
	Point goal = Point(1250, 700);

	// for image1k
	// Point start = Point(5015, 2142);
	// Point goal = Point(546, 2730);
	//Point goal = Point(1120, 552);

	Point point = Point(100, 100);

	// --- testing ---

	StagePoint sP(start, Point(0, 0), 0.0, 0.0);
	std::vector<StagePoint> close;
	std::map<Point, StagePoint> mapPath;



	auto timer = std::chrono::steady_clock::now();
	auto resPath = Lian(start, goal, mImg, mImgSource, 5, 270);

	for (auto&& point : resPath) {

		cv::circle(mImg.getMap(), cv::Point(point.x, point.y), 10, cv::Scalar(100, 100, 100), 2);
	}
	std::cout << "Time code -> " << std::chrono::duration <double, std::milli>(std::chrono::steady_clock::now() - timer).count() << std::endl;

	// --- end testing----

	imshow("Display window", img);
	int k = cv::waitKey(0);

	return 0;
}