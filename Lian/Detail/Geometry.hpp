#pragma once

#include <vector>

#define _USE_MATH_DEFINES
#include <math.h>

#include "../Point.hpp"
#include "Vector.hpp"

namespace Algorithms {

	namespace Graph {

		namespace Geometry {

			Vector makeWindVector(double scale, double speed, Vector direction) {
				double offset = speed * scale / 1.5 / direction.getMagnitude();
				direction *= offset;
				return direction;
			}

			double distanceBetweenPoints(Point p1, Point p2) {

				return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
			}

			double angleBetweenVectors(Point a1, Point a2, Point b1, Point b2) {


				Vector a(a2.x - a1.x, a2.y - a1.y, a2.z - a1.z);
				Vector b(b2.x - b1.x, b2.y - b1.y, b2.z - b1.z);

				double angle = acos(a.scalar(b) / (a.getMagnitude() * b.getMagnitude()));

				return abs(angle * 180 / M_PI);
			}

			std::vector<Point> lineOfSight(Point p1, Point p2) {

				int nb_points = distanceBetweenPoints(p1, p2);

				double x_spacing = (p2.x - p1.x) / double(nb_points + 1);
				double y_spacing = (p2.y - p1.y) / double(nb_points + 1);
				double z_spacing = (p2.z - p1.z) / double(nb_points + 1);


				std::vector<Point> points;
				for (int i = 1; i < nb_points + 1; ++i) {
					points.emplace_back(int(p1.x + i * x_spacing), int(p1.y + i * y_spacing), int(p1.z + i * z_spacing));
				}

				return points;
			}


			std::vector<std::pair<int, int>> midpoint2d(int X, int Y, int r) {

				
				std::vector<std::pair<int, int>> points;
				if (r == 0) {
					return points;
				}

				int x_centre = X;
				int y_centre = Y;

				int x = r;
				int y = 0;
				points.emplace_back(x + x_centre, y + y_centre);
				if (r == 0) {
					return points;
				}
				if (r > 0) {
					points.emplace_back(-x + x_centre, -y + y_centre);
					points.emplace_back(y + x_centre, x + y_centre);
					points.emplace_back(-y + x_centre, -x + y_centre);
				}

				int P = 1 - r;

				while (x > y) {

					y += 1;
					if (P <= 0) {
						P = P + 2 * y + 1;
					}
					else {
						x -= 1;
						P = P + 2 * y - 2 * x + 1;
					}

					if (x < y)
						break;

					points.emplace_back(x + x_centre, y + y_centre);
					points.emplace_back(-x + x_centre, y + y_centre);
					points.emplace_back(x + x_centre, -y + y_centre);
					points.emplace_back(-x + x_centre, -y + y_centre);

					if (x != y) {
						points.emplace_back(y + x_centre, x + y_centre);;
						points.emplace_back(-y + x_centre, x + y_centre);
						points.emplace_back(y + x_centre, -x + y_centre);
						points.emplace_back(-y + x_centre, -x + y_centre);
					}

				}

				return points;
			}

			std::vector<Point> midpoint(Point point, int r) {
				std::vector<Point> points;
				std::vector<std::pair<int, int>> c1 = midpoint2d(point.x, point.z, r);
				for (int i = 0; i < c1.size(); i++) {
					points.emplace_back(c1[i].first, point.y, c1[i].second);
				}
				std::vector<std::pair<int, int>> c2 = midpoint2d(point.y, point.z, r);
				for (int i = 0; i < c2.size(); i++) {
					points.emplace_back(point.x, c2[i].first, c2[i].second);
				}
				std::vector<int> y_checked;
				for (int i = 0; i < c2.size(); i++) {
					int x = c2[i].first;
					int y = c2[i].second;
					if (std::find(y_checked.begin(), y_checked.end(), x) != y_checked.end()) {
						continue;
					}
					else {
						y_checked.push_back(x);
					}

					int radius = y - point.z;

					std::vector<std::pair<int, int>> circle = midpoint2d(point.x, point.z, radius);
					for (int i = 0; i < circle.size(); i++) {
						points.emplace_back(int(circle[i].first), int(x), int(circle[i].second));
					}
				}
				return points;
			}

		}
	}
}