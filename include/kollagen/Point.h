#ifndef POINT_H
#define POINT_H

namespace kollagen
{

struct Point {
  Point(int x, int y) : x(x), y(y){};
  const int x;
  const int y;
  bool operator==(const Point &rhs) const { return x == rhs.x && y == rhs.y; };
};

}  // namespace kollagen
#endif /* !POINT_H */
