#pragma once

#include <opencv2/opencv.hpp>

#include "Map.hpp"
#include "Vector.hpp"
#include "LianFunctions.hpp"
#include "../Point.hpp"

namespace Algorithms {

	namespace WindEffects {

		using Algorithms::Graph::Map::Map;
		using Algorithms::Graph::Geometry::Vector;
		using Algorithms::Graph::Geometry::Point;
		using Algorithms::Graph::Geometry::lineOfSight;
		using Algorithms::Graph::LianFunctions::validPath;

		cv::Mat makeShade(Map<cv::Mat> image, Vector wind, double scale) {

			cv::Mat copy = image.getMap().clone();

			double k = scale;
			int x = wind.getX();
			int y = wind.getY();

			for (int i = 0; i < copy.cols; i++) {
				for (int j = 0; j < copy.rows; j++) {
					//-x + y;
					int px = i - k * x;
					int py = j + k * y;
					
					if (i - k * x < 0) {
						px = 0;
					}
					if (i - k * x >= copy.cols) {
						px = copy.cols;
					}
					if (j + k * y < 0) {
						py = 0;
					}
					if (j + k * y >= copy.rows) {
						py = copy.rows;
					}
					if (copy.at<uchar>(cv::Point(i, j)) == 0) {
						continue;
					}
					Point p1(px, py);
					Point p2(i, j);
					if (copy.at<uchar>(cv::Point(px, py)) == 0) {
						copy.at<uchar>(cv::Point(i, j)) = 100;
					}
					if (!validPath(lineOfSight(p1, p2), image)) {
						copy.at<uchar>(cv::Point(i, j)) = 100;
					}
				}
			}
			return copy;
		}
	}
}