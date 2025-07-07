#pragma once

#include "../Point.hpp"

namespace Algorithms {

	namespace Graph {

		namespace Geometry {

			struct StagePoint {
				Point point;
				Point parent;

				double distance;
				double sumAngles;
				double wind;

				StagePoint(Point point_, Point parent_, double distance_, double sumAngles_, double wind_)
					: point(point_), parent(parent_), distance(distance_), sumAngles(sumAngles_), wind(wind_) {};

				auto operator<=>(const StagePoint& rhs) const = default;
			};

		}
	}
}