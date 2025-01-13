#pragma once

namespace Algorithms {

	namespace Graph {

		namespace Geometry {

			struct Point {

				int x, y, z;

				auto operator<=>(const Point& rhs) const = default;
			};
		}
	}
}