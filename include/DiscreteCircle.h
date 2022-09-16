#include <vector>

#include "Point.h"

#ifndef DISCRETECIRCLE_H
#define DISCRETECIRCLE_H

class DiscreteCircle {
public:
  explicit DiscreteCircle(int radius) : radius_(radius) {
    for (int x = -radius; x <= radius; ++x) {

      int y_limit = radius * radius - x * x;
      for (int y = 0; y * y <= y_limit; ++y) {
        coordinate_list_.emplace_back(x, y);
        if (y > 0) {
          coordinate_list_.emplace_back(x, -y);
        }
      }
    }
  };

  [[nodiscard]] int radius() const { return radius_; };

  [[nodiscard]] std::vector<Point> coordinates() const {
    return coordinate_list_;
  };

  [[nodiscard]] std::vector<int> x_list() const {
    std::vector<int> x_list;
    for (auto coordinate : coordinate_list_) {
      x_list.emplace_back(coordinate.x);
    }
    return x_list;
  };
  [[nodiscard]] std::vector<int> y_list() const {
    std::vector<int> y_list;
    for (auto coordinate : coordinate_list_) {
      y_list.emplace_back(coordinate.y);
    }
    return y_list;
  };

  auto begin() { return coordinate_list_.begin(); }
  auto end() { return coordinate_list_.end(); }
  [[nodiscard]] auto cbegin() const { return coordinate_list_.begin(); }
  [[nodiscard]] auto cend() const { return coordinate_list_.end(); }
  [[nodiscard]] auto begin() const { return coordinate_list_.begin(); }
  [[nodiscard]] auto end() const { return coordinate_list_.end(); }

private:
  const int radius_;
  std::vector<Point> coordinate_list_;
};


#endif /* !DISCRETECIRCLE_H */
