#  LianStar
## Description
Algorithm Lian modification. Changes made to the original:
- angle minimization
- constant wind considered
- work in 3d added

## Usage
To work with your own images change these lines in Source.cpp:
~~~cpp
#define PATH_IMG "resources/irk.png"
#define PATH_IMG_SOURCE "resources/irk.png"
~~~
~~~cpp
Point start(14, 5, 7);
Point goal(450, 450, 450);
~~~
~~~cpp
auto resPath = Lian(start, goal, matrix, size_x, size_y, size_z, 80, 25);
~~~
