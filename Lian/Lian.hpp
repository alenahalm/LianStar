#pragma once

#include <opencv2/opencv.hpp>

#include <iostream>
#include <algorithm>
#include <chrono>

#include <vector>
#include <set>
#include <unordered_set>
#include <float.h>

#include "Point.hpp"
#include "Detail/StagePoint.hpp"
#include "Detail/Comparator.hpp"
#include "detail/Hasher.hpp"
#include "Detail/Path.hpp"
#include "Detail/Map.hpp"
#include "Detail/Vector.hpp"

#include "detail/Geometry.hpp"
#include "Detail/LianFunctions.hpp"

#define DIR_RESULTS "./results/"

#define K_DELTA 1
#define K_ANGLE 0

namespace Algorithms {

	namespace Graph {

		namespace Lian {

			using std::vector, std::set, std::unordered_set;
			using Algorithms::Graph::Map::Map;
			using namespace Geometry;
			using LianFunctions::Expand;
			using LianFunctions::unwindingPath;
			using LianFunctions::showImageThread;
			using LianFunctions::saveImage;
			using LianFunctions::drawStateOnImage;
			using LianFunctions::logConsole;
			using LianFunctions::logFile;

			vector<Point> Lian(Point start_, Point goal_, Map<cv::Mat> img, Map<cv::Mat> drawImg, int deltaDist, int deltaAngle, Vector wind) {

				Comparator::goal = goal_;
				Comparator::KDelta = K_DELTA;
				Comparator::KAngle = K_ANGLE;


				set<StagePoint, Comparator::ComparatorStagePoint> OPEN;
				unordered_set<StagePoint, Hasher::StagePointHasher> CLOSE;

				//OPEN.reserve(100000);
				CLOSE.reserve(100000);

				StagePoint start(start_, Point(0, 0), 0.0, 0.0, 0.0),
					goal(goal_, Point(0, 0), DBL_MAX, DBL_MAX, 0.0);
				OPEN.insert(start);

				std::map<Point, StagePoint> mapPath;

				int itCounter{ 0 };
				//auto itCurrent = OPEN.begin();	// delete it and replace on obj downer
				//auto itOpenSP = OPEN.begin();	// temp obj for find_if in loop
				StagePoint currentSPoint = start;

				vector<StagePoint> res;

				int pathCounter{ 0 };
				int totalQPath{ 0 };
				Path bestPath({}, DBL_MAX, DBL_MAX, 0.0);

				bool isAction{ true };
				//std::thread t([&isAction, start_, goal_, &currentSPoint, img, &OPEN, &CLOSE, &mapPath]() {
				//showImageThread(isAction, start_, goal_, currentSPoint.point, img, OPEN, CLOSE, mapPath);
					//});

				auto startTimer = std::chrono::steady_clock::now();
				auto timer = std::chrono::steady_clock::now();

				while (!OPEN.empty()) {

					auto current = OPEN.extract(OPEN.begin());
					currentSPoint = current.value();
					//if (wind.getMagnitude() != 0) {
						//std::cout << currentSPoint.wind << std::endl;
					//}
					

					if (currentSPoint.point == goal.point) {

						if (currentSPoint.sumAngles <= bestPath.sumAngles) {

							++pathCounter;

							auto points = unwindingPath(mapPath, start_, goal_);	// save path
							bestPath = Path(points, currentSPoint.distance, currentSPoint.sumAngles, currentSPoint.wind);

							logConsole(bestPath);	// log in console

							double timeCode = std::chrono::duration <double, std::milli>(std::chrono::steady_clock::now() - startTimer).count() / 1000;	// time in seconds
							logFile(DIR_RESULTS + std::string("Path_") + std::to_string(pathCounter) + ".txt", bestPath, deltaDist, deltaAngle, timeCode, K_DELTA, K_ANGLE);	// log in file

							auto imgPathSource = drawStateOnImage(start_, goal_, currentSPoint.point, drawImg, false, {}, {}, mapPath);
							saveImage(DIR_RESULTS + std::string("Path_") + std::to_string(pathCounter) + ".bmp", imgPathSource);	// save source image with path

							auto imgPath = drawStateOnImage(start_, goal_, currentSPoint.point, img, true, {}, {}, mapPath);
							saveImage(DIR_RESULTS + std::string("Path_") + std::to_string(pathCounter) + ".png", imgPath);	// save processing image with path

							return bestPath.points;
						}
						else {
							std::cout << "Path found, but skipped" << std::endl;
						}
						++totalQPath;
						std::cout << "Total path found quantity: " << totalQPath << std::endl;
					}

					Expand(start, img, currentSPoint, deltaDist, deltaAngle, wind, OPEN, CLOSE, goal_, mapPath);

					CLOSE.insert(currentSPoint);

					if (std::chrono::duration <double, std::milli>
						(std::chrono::steady_clock::now() - timer).count() > 1000) {

						std::cout << "Iteration -> " << itCounter << std::endl;
						// std::cout << "Offset -> " << currentSPoint.wind << std::endl;
						showImageThread(isAction, start_, goal_, currentSPoint.point, drawImg, OPEN, CLOSE, mapPath);
						timer = std::chrono::steady_clock::now();
					}

					//if ((itCounter & 0x9B))	// log itCounter every 100 iterations
						//std::cout << "Iteration -> " << itCounter << std::endl;

					++itCounter;

				}

				isAction = false;
				//t.join();

				return bestPath.points;
			}

		}

	}

}