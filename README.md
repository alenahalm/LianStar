#  LianStar
## Description
Algorithm Lian modification. Changes made to the original:
- angle minimization
- constant wind considered
- work in 3d added

## Usage
Добавьте собственные изображения в папку _resources_.

Путь к собственным изображениям указывается в файле _Source.cpp_:
~~~cpp
#define PATH_IMG "resources/irk.png"
#define PATH_IMG_SOURCE "resources/irk.png"
~~~
Переменные, которые нужно изменить выделены комментариями //INIT:
~~~cpp
// INIT

  cv::Mat rawImg = cv::imread(PATH_IMG, cv::IMREAD_COLOR);
  cv::Mat rawImgSource = cv::imread(PATH_IMG_SOURCE, cv::IMREAD_COLOR);

  // Points to find path for
  Point start = Point(190, 310);
  Point goal = Point(1287, 689);

  // Params for path
  int delta = 25;
  int angle = 25;

  // Is new version of algorithm? For basic Lian star = false;
  bool star = true;

  // Wind data
  double scale = 3; // pixels in 1m
  Vector direction(1, -1); // wind direction
  double speed = 0; // wind speed

  // Name for output files
  string note = "irk_";


// END INIT
~~~

Координаты точек:
~~~cpp
// Points to find path for
Point start = Point(190, 310);
Point goal = Point(1287, 689);
~~~

Параметры delta (шаг), angle (ограничение по углу). Delta реккомендуется 25.
~~~cpp
// Params for path
int delta = 25;
int angle = 25;
~~~
Значение star, остается неизменным. При star=false, отключается минимизация угла.
~~~cpp
// Is new version of algorithm? For basic Lian star = false;
bool star = true;
~~~
Данные для ветра: масштаб, направление в виде вектора, скорость.
~~~cpp
// Wind data
double scale = 3; // pixels in 1m
Vector direction(1, -1); // wind direction
double speed = 0; // wind speed
~~~
Название файлов с результатами.
~~~cpp
// Name for output files
string note = "irk_";
~~~
