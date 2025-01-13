#include <iostream>
#include <array>
#include <chrono>

#include <opencv2/opencv.hpp>

#include "Lian/Point.hpp"

#include "Lian/Lian.hpp"
#include "Lian/Detail/Geometry.hpp"
#include "Lian/Detail/Map.hpp"
#include "Lian/Detail/LianFunctions.hpp"
#include "Lian/Detail/Vector.hpp"


#define PATH_IMG "resources/irk.png"
#define PATH_IMG_SOURCE "resources/irk.png"



using namespace std;
using Algorithms::Graph::Lian::Lian;
using namespace Algorithms::Graph::Geometry;
using namespace Algorithms::Graph::Map;
using Algorithms::Graph::LianFunctions::Expand;


using Algorithms::Graph::Lian::lineOfSight;
using Algorithms::Graph::LianFunctions::validPath;



int main() {

	// --- initializing the matrix ---

	int size_x = 1000, size_y = 1000, size_z = 1000;

	//double matrix[size][size][size];

	double*** matrix = new double** [size_x];
	for (int i = 0; i < size_x; ++i) {
		matrix[i] = new double* [size_y];
		for (int j = 0; j < size_y; ++j) {
			matrix[i][j] = new double[size_z]();
		}
	}

	for (int i = 0; i < size_x; i++) {
		for (int j = 0; j < size_y; j++) {
			for (int k = 0; k < size_z; k++) {
				if (i > 200 && i < 350 && j > 200 && j < 350 && k > 200 && k < 350){
					matrix[i][j][k] = 1;
				}
				else {
					matrix[i][j][k] = 0;
				}
			}
		}
	}
	

	// --- variables ---

	Point start(14, 5, 7);
	Point goal(450, 450, 450);

	// --- testing ---

	
	auto timer = std::chrono::steady_clock::now();
	auto resPath = Lian(start, goal, matrix, size_x, size_y, size_z, 80, 25);


	

	std::cout << "Time code -> " << std::chrono::duration <double, std::milli>(std::chrono::steady_clock::now() - timer).count() << std::endl;
	
	// --- end testing----
	
	int k = cv::waitKey(0);

	return 0;
}