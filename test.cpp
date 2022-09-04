#include <boost/polygon/voronoi.hpp>

using namespace boost::polygon;

struct Point {
  int a;
  int b;
  Point(int x, int y) : a(x), b(y) {}
};

struct Segment {
  Point p0;
  Point p1;
  Segment(int x1, int y1, int x2, int y2) : p0(x1, y1), p1(x2, y2) {}
};

namespace boost {
namespace polygon {

template <>
struct geometry_concept<Point> {
  typedef point_concept type;
};

template <>
struct point_traits<Point> {
  typedef int coordinate_type;

  static inline coordinate_type get(
      const Point& point, orientation_2d orient) {
    return (orient == HORIZONTAL) ? point.a : point.b;
  }
};

template <>
struct geometry_concept<Segment> {
  typedef segment_concept type;
};

template <>
struct segment_traits<Segment> {
  typedef int coordinate_type;
  typedef Point point_type;

  static inline point_type get(const Segment& segment, direction_1d dir) {
    return dir.to_int() ? segment.p1 : segment.p0;
  }
};
}  // polygon
}  // boost


int main()
{
	std::vector<Point> points;
	points.push_back(Point(0, 0));
	points.push_back(Point(1, 6));
	std::vector<Segment> segments;
	segments.push_back(Segment(-4, 5, 5, -1));
	segments.push_back(Segment(3, -11, 13, -1));
	voronoi_diagram<double> vd;
	construct_voronoi(points.begin(), points.end(), segments.begin(), segments.end(), &vd);
}
