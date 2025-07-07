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


// path to images
#define PATH_IMG "resources/irk.png" // image to analyze
#define PATH_IMG_SOURCE "resources/irk.png" // image to draw on


using namespace std;
using Algorithms::Graph::Lian::Lian;
using namespace Algorithms::Graph::Geometry;
using namespace Algorithms::Graph::Map;
using Algorithms::Graph::LianFunctions::Expand;
using Algorithms::WindEffects::makeShade;

int main() {


	// INIT

		cv::Mat rawImg = cv::imread(PATH_IMG, cv::IMREAD_COLOR);
		cv::Mat rawImgSource = cv::imread(PATH_IMG_SOURCE, cv::IMREAD_COLOR);

		// Points to find path for
		Point start = Point(190, 310);
		Point goal = Point(1287, 689);

		// Params for path
		int delta = 25;
		int angle = 25;

		// Is new version of algorithm? For basic Lian star = false;
		bool star = true;

		// Wind data
		double scale = 3; // pixels in 1m
		Vector direction(1, -1); // wind direction
		double speed = 0; // wind speed
	
		// Name for output files
		string note = "irk_";


	// END INIT
	if (rawImg.empty()) {

		std::cerr << "Image not found!" << std::endl;

		return -1;
	}

	cv::Mat img;

	cv::cvtColor(rawImg, img, cv::COLOR_BGR2GRAY);

	img.setTo(255, img > 200);
	img.setTo(0, img != 255);


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
	auto resPath = Lian(start, goal, mImg, mImgSource, delta, angle, wind, scale, star, note);

	for (auto&& point : resPath) {
		cv::circle(mImg.getMap(), cv::Point(point.x, point.y), 3, cv::Scalar(100, 100, 100), -1);
	}

	std::cout << "Time code -> " << std::chrono::duration <double, std::milli>(std::chrono::steady_clock::now() - timer).count() << std::endl;

	// --- end testing----

	imshow("Display window", img);
	int k = cv::waitKey(0);

	return 0;
}