#ifndef WALKER_IMPL_H
#define WALKER_IMPL_H

#include "Walker.h"
#include "Functions.h"

namespace kollagen
{

inline double Walker::theta_as_double() const {
  return scale_int_angle(theta_);
}
inline double Walker::x_as_double() const { return scale_int_length(x_); }
inline double Walker::y_as_double() const { return scale_int_length(y_); }

inline double Walker::scale_int_length(int length) const {
  return length * stepsize_ / step_fragment_;
}

inline double Walker::scale_int_angle(int angle) {
  return normalize_angle(angle * 0.5 * M_PI);
}

inline int Walker::scale_double_length(double length) const {
  return round_to_int(length * step_fragment_ / stepsize_);
}

inline int Walker::scale_double_angle(double angle) {
  return (round_to_int(normalize_angle(angle) / 0.5 / M_PI) + 4) % 4;
}

inline Walker::Walker(double x, double y, double theta, double stepsize,
                      double sigma_pos, double sigma_ang, int step_fragment,
                      int seed, int ID, double I11est, double I22est,
                      double I33est, bool allow_retrocession)
    : ID_(ID), x0_(x), y0_(y), theta0_(theta), step_fragment_(step_fragment),
      stepsize_(stepsize), x_(scale_double_length(x)),
      y_(scale_double_length(y)), theta_(scale_double_angle(theta)),
      std_dev_pos_(sigma_pos), std_dev_ang_(sigma_ang),
      I11est_(I11est > 0 ? I11est : 1 / std::pow(sigma_pos, 2)),
      I22est_(I22est > 0 ? I22est : 1 / std::pow(sigma_pos, 2)),
      I33est_(I33est > 0 ? I33est : 1 / std::pow(sigma_pos, 2)), X_({x}),
      Y_({y}), Theta_({theta}), drx_(x), dry_(y), drtheta_(theta), Drx_({x}),
      Dry_({y}), Drtheta_({theta}), rng(seed, allow_retrocession){};

inline Walker::Walker(WalkerParams p)
    : ID_(p.ID), x0_(p.x), y0_(p.y), theta0_(p.theta),
      step_fragment_(p.step_fragment), stepsize_(p.stepsize),
      x_(scale_double_length(p.x)), y_(scale_double_length(p.y)),
      theta_(scale_double_angle(p.theta)), std_dev_pos_(p.sigma_pos),
      std_dev_ang_(p.sigma_ang), I11est_(p.I11est), I22est_(p.I22est),
      I33est_(p.I33est), X_({p.x}), Y_({p.y}), Theta_({p.theta}), drx_(p.x),
      dry_(p.y), drtheta_(p.theta), Drx_({p.x}), Dry_({p.y}),
      Drtheta_({p.theta}), rng(p.seed, p.allow_retrocession){};

inline void Walker::step(int N) {
  for (int i = 0; i < N; ++i) {
    this->step();
  }
};

inline int Walker::random_turn() {
  int dtheta = draw_int();
  return dtheta;
};

inline void Walker::dead_reckon(double odom_ang, double odom_pos) {
  drtheta_ += odom_ang;
  drtheta_ = normalize_angle(drtheta_);
  drx_ += odom_pos * std::cos(drtheta_);
  dry_ += odom_pos * std::sin(drtheta_);

  Drx_.emplace_back(drx_);
  Dry_.emplace_back(dry_);
  Drtheta_.emplace_back(drtheta_);
}

inline std::tuple<int, int> Walker::position_change_in_global_frame() const {
  double theta = theta_as_double();
  int dx = round_to_int(std::cos(theta));
  int dy = round_to_int(std::sin(theta));
  return {dx, dy};
}

inline void Walker::simulate_steps(int dtheta) {
  for (int i = 0; i < step_fragment_; ++i) {
    if (i == 0) {
      theta_ = (theta_ + dtheta) % 4;
    }

    auto [int_dx, int_dy] = position_change_in_global_frame();
    x_ += int_dx;
    y_ += int_dy;

    double dx = scale_int_length(int_dx);
    double dy = scale_int_length(int_dy);

    X_.emplace_back(x_as_double());
    Y_.emplace_back(y_as_double());
    Theta_.emplace_back(theta_as_double());

    double odom_pos = std::hypot(dx, dy) + std_dev_pos_ * draw_normal();
    double odom_ang{};
    if (i == 0) {
      odom_ang = scale_int_angle(dtheta) + std_dev_ang_ * draw_normal();
      delta_theta_.emplace_back(dtheta);
    } else if (i > 0) {
      odom_ang = std_dev_ang_ * draw_normal();
      delta_theta_.emplace_back(0);
    }
    delta_x_.emplace_back(int_dx);
    delta_y_.emplace_back(int_dy);

    odom_pos_.emplace_back(odom_pos);
    odom_ang_.emplace_back(odom_ang);

    dead_reckon(odom_ang_.back(), odom_pos_.back());
  }
};

inline void Walker::step() {
  int dtheta = random_turn();
  simulate_steps(dtheta);
};

inline void Walker::step_left() { simulate_steps(1); };

inline void Walker::step_right() { simulate_steps(-1); };

inline void Walker::step_forward() { simulate_steps(0); };

inline int Walker::writeg2o(const std::string &filename, const bool &_3D, const bool &exact_information_matrix) const {
  auto file = open_file(filename);

  if (!file) {
    return EXIT_FAILURE;
  }

  write_vertices(file, {Drx(), Dry(), Drtheta()}, _3D);

  write_odometry(file, *this, _3D, exact_information_matrix);

  write_loop_closures(file, LC_, _3D);

  return EXIT_SUCCESS;
};

inline int Walker::write_ground_truth_as_TUM(const std::string &filename) const {
  std::filesystem::path path(filename);
  if (path.extension() != ".tum") {
    path += ".tum";
  }
  std::cout << "Saving ground truth to " << path.string() << '\n';

  std::ofstream file(path.string(), std::ios_base::out);
  if (!file) {
    return EXIT_FAILURE;
  }

  write_TUM_vertices(file, {X(), Y(), Theta()});

  return EXIT_SUCCESS;
};

inline void Walker::translate(const int &x, const int &y) {
  std::for_each(X_.begin(), X_.end(), [&x](double &n) { n += x; });
  std::for_each(Drx_.begin(), Drx_.end(), [&x](double &n) { n += x; });

  std::for_each(Y_.begin(), Y_.end(), [&y](double &n) { n += y; });
  std::for_each(Dry_.begin(), Dry_.end(), [&y](double &n) { n += y; });
  x0_ += x;
  y0_ += y;
}

inline void Walker::translate(const Point &p) { translate(p.x, p.y); }

}  // namespace kollagen
#endif /* !WALKER_IMPL_H */
