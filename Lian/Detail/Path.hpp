#pragma once

#include <vector>

#include "../Point.hpp"

namespace Algorithms {

	namespace Graph {

		namespace Geometry {

			struct Path {
				std::vector<Point> points;

				double distance;
				double sumAngles;
				double windSum;

				Path(std::vector<Point> points, double distance, double sumAngles, double windSum)
					: points(points), distance(distance), sumAngles(sumAngles), windSum(windSum) {}
			};
		}
	}
}