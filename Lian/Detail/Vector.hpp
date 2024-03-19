#pragma once

namespace Algorithms {
	
	namespace Graph {
		
		namespace Geometry {

			struct Vector {

				double x;
				double y;

				Vector(double x_, double y_) : x(x_), y(y_) {};
				
				auto operator-=(const Vector& rhs) {
					x -= rhs.x;
					y -= rhs.y;
				}

				auto operator+=(const Vector& rhs) {
					x += rhs.x;
					y += rhs.y;
				}

				auto operator - (const Vector& rhs) {
					return Vector(x - rhs.x, y - rhs.y);
				}

				auto operator + (const Vector& rhs) {
					return Vector(x + rhs.x, y + rhs.y);
				}

				double getMagnitude() {
					return std::sqrt(x * x + y * y);
				}

				double getX() {
					return x;
				}
				double getY() {
					return y;
				}

				void print() {
					std::cout << "(" << x << ", " << y << ")" << std::endl;
				}
			};
		}
	}
}