#pragma once

namespace Algorithms {
	
	namespace Graph {
		
		namespace Geometry {

			struct Vector {

				double x;
				double y;
				double z;

				Vector(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {};
				
				auto operator-=(const Vector& rhs) {
					x -= rhs.x;
					y -= rhs.y;
					z -= rhs.z;
				}

				auto operator+=(const Vector& rhs) {
					x += rhs.x;
					y += rhs.y;
					z += rhs.z;
				}
				auto operator *= (double num) {
					x *= num;
					y *= num;
					z *= num;
				}

				auto operator - (const Vector& rhs) {
					return Vector(x - rhs.x, y - rhs.y, z - rhs.z);
				}

				auto operator + (const Vector& rhs) {
					return Vector(x + rhs.x, y + rhs.y, z + rhs.z);
				}

				double getMagnitude() {
					return std::sqrt(x * x + y * y + z * z);
				}

				double getX() {
					return x;
				}
				double getY() {
					return y;
				}
				double getZ() {
					return z;
				}

				double scalar(const Vector& rhs) {
					return x * rhs.x + y * rhs.y + z * rhs.z;
				}

				std::string to_string() {
					return "(" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")";
				}

				void print() {
					std::cout << "(" << x << ", " << y << ", " << z << ")" << std::endl;
				}
			};
		}
	}
}