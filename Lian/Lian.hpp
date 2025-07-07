#pragma once

#include "opencv2/opencv.hpp"

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
#include "Detail/WindEffect.hpp"

#include "detail/Geometry.hpp"
#include "Detail/LianFunctions.hpp"

#define DIR_RESULTS "./results/eLian/"

// initializing priority factors 
#define K_DELTA 1 // distance
#define K_ANGLE 0.1 // angles
#define K_WIND 0 // wind

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
			using WindEffects::makeShade;

			vector<Point> Lian(Point start_, Point goal_, Map<cv::Mat> img, Map<cv::Mat> drawImg, int deltaDist, int deltaAngle, Vector wind, double scale, bool star, std::string note) {

				// assigning user-input values to factors
				Comparator::goal = goal_;
				Comparator::KDelta = K_DELTA;
				if (star) {
					Comparator::KAngle = K_ANGLE;
				}
				else {
					Comparator::KAngle = 0;
				}
				Comparator::KWind = K_WIND;

				// INIT
				set<StagePoint, Comparator::ComparatorStagePoint> OPEN;
				unordered_set<StagePoint, Hasher::StagePointHasher> CLOSE;

				//OPEN.reserve(100000);
				CLOSE.reserve(100000);

				StagePoint start(start_, Point(0, 0), 0.0, 0.0, 0.0),
					goal(goal_, Point(0, 0), DBL_MAX, DBL_MAX, 0.0);
				OPEN.insert(start);

				std::map<Point, StagePoint> mapPath;

				int itCounter{ 0 };
				StagePoint currentSPoint = start;

				vector<StagePoint> res;

				int pathCounter{ 0 };
				int totalQPath{ 0 };
				Path bestPath({}, DBL_MAX, DBL_MAX, 0.0);

				bool isAction{ true };


				auto startTimer = std::chrono::steady_clock::now();
				auto timer = std::chrono::steady_clock::now();

				while (!OPEN.empty()) {

					// get the next point
					auto current = OPEN.extract(OPEN.begin());
					currentSPoint = current.value();


					// check if reached the goal
					if (currentSPoint.point == goal.point) {

						if (currentSPoint.sumAngles <= bestPath.sumAngles) {

							++pathCounter;
							/*if (star) {
								note += "star";
							}
							else {
								note += "lian";
							}*/

							auto points = unwindingPath(mapPath, start_, goal_);	// save path
							bestPath = Path(points, currentSPoint.distance, currentSPoint.sumAngles, currentSPoint.wind);

							logConsole(bestPath);	// log in console

							double timeCode = std::chrono::duration <double, std::milli>(std::chrono::steady_clock::now() - startTimer).count() / 1000;	// time in seconds
							logFile(DIR_RESULTS + note + ".txt", bestPath, deltaDist, deltaAngle, wind, timeCode, K_DELTA, K_ANGLE);	// log in file

							//auto imgPathSource = drawStateOnImage(start_, goal_, currentSPoint.point, drawImg, false, {}, {}, mapPath);
							//saveImage(DIR_RESULTS + note + ".bmp", imgPathSource);	// save source image with path

							auto imgPath = drawStateOnImage(start_, goal_, currentSPoint.point, img, true, {}, {}, mapPath);
							saveImage(DIR_RESULTS + note + ".png", imgPath);	// save processing image with path

							return bestPath.points;
						}
						else {
							std::cout << "Path found, but skipped" << std::endl;
						}
						++totalQPath;
						std::cout << "Total path found quantity: " << totalQPath << std::endl;
					}

					// build the circle around current point
					Expand(start, img, currentSPoint, deltaDist, deltaAngle, wind, OPEN, CLOSE, goal_, mapPath, star);

					CLOSE.insert(currentSPoint);

					// showing progress on image
					if (std::chrono::duration <double, std::milli>
						(std::chrono::steady_clock::now() - timer).count() > 1000) {

						std::cout << "Iteration -> " << itCounter << std::endl;
						// std::cout << "Offset -> " << currentSPoint.wind << std::endl;
						showImageThread(isAction, start_, goal_, currentSPoint.point, drawImg, OPEN, CLOSE, mapPath);
						timer = std::chrono::steady_clock::now();
					}



					++itCounter;

				}

				isAction = false;
				//t.join();

				return bestPath.points;
			}

		}

	}

}