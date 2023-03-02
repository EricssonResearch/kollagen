#ifndef WALKER_H
#define WALKER_H

#include <numeric>
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <tuple>
#include <vector>

#include "DiscreteCircle.h"
#include "InterLCParams.h"
#include "LCParams.h"
#include "Loopclosure.h"
#include "MultiLoopclosure.h"
#include "RNGenerator.h"
#include "WalkerParams.h"

#include "utils.hpp"

#define _USE_MATH_DEFINES

namespace kollagen
{

class Walker {
public:
  virtual ~Walker() = default;
  Walker(const Walker &) = default;
  Walker(Walker &&) = default;
  Walker &operator=(const Walker &) = default;
  Walker &operator=(Walker &&) = default;

  explicit Walker(WalkerParams params = WalkerParams());

  Walker(double x, double y, double theta, double stepsize, double sigma_pos,
         double sigma_ang, int step_fragment, int seed, int ID, double I11est,
         double I22est, double I33est, bool allow_retrocession = true);

  void step();
  void step(int N);

  void step_left();
  void step_right();
  void step_forward();

  int writeg2o(const std::string &filename, const bool &_3D = false, const bool &exact_information_matrix = false) const;
  int write_ground_truth_as_TUM(const std::string &filename) const;

  [[nodiscard]] int step_fragment() const { return step_fragment_; };
  [[nodiscard]] double stepsize() const { return stepsize_; };

  [[nodiscard]] WalkerID ID() const { return ID_; };

  [[nodiscard]] size_t size() const { return X_.size(); };

  [[nodiscard]] std::vector<double> Drx() const { return Drx_; };
  [[nodiscard]] std::vector<double> Dry() const { return Dry_; };
  [[nodiscard]] std::vector<double> Drtheta() const { return Drtheta_; };

  [[nodiscard]] double Drx(int i) const { return Drx_.at(i); };
  [[nodiscard]] double Dry(int i) const { return Dry_.at(i); };
  [[nodiscard]] double Drtheta(int i) const { return Drtheta_.at(i); };

  [[nodiscard]] double X(int i) const { return X_.at(i); };
  [[nodiscard]] double Y(int i) const { return Y_.at(i); };
  [[nodiscard]] double Theta(int i) const { return Theta_.at(i); };
  [[nodiscard]] Loopclosure LC(int i) const { return LC_.at(i); };

  [[nodiscard]] std::vector<double> X() const { return X_; };
  [[nodiscard]] std::vector<double> Y() const { return Y_; };
  [[nodiscard]] std::vector<double> Theta() const { return Theta_; };
  [[nodiscard]] std::vector<Loopclosure> LC() const { return LC_; };

  [[nodiscard]] int seed() const { return rng.seed(); };

  [[nodiscard]] WalkerParams Params() const {
    return {x0_,          y0_,          theta0_,        stepsize_,
            std_dev_pos_, std_dev_ang_, step_fragment_, rng.seed(),
            ID_,          I11est_,      I22est_,        I33est_};
  };

  [[nodiscard]] double odom_pos(int i) const { return odom_pos_.at(i+1); };
  [[nodiscard]] double odom_ang(int i) const { return odom_ang_.at(i+1); };

  [[nodiscard]] double std_dev_pos() const { return std_dev_pos_; };
  [[nodiscard]] double std_dev_ang() const { return std_dev_ang_; };

  [[nodiscard]] std::vector<double> odom_pos() const {
    return {odom_pos_.begin() + 1, odom_pos_.end()};
  };
  [[nodiscard]] std::vector<double> odom_ang() const {
    return {odom_ang_.begin() + 1, odom_ang_.end()};
  };

  [[nodiscard]] virtual double draw_normal() { return rng.draw_normal(); };
  [[nodiscard]] virtual double draw_uniform() { return rng.draw_uniform(); };
  [[nodiscard]] virtual int draw_int() { return rng.draw_int(); };

  void addLC(const Loopclosure &LC) { LC_.emplace_back(LC); };

  void translate(const int &x, const int &y);
  void translate(const Point &p);

  [[nodiscard]] double scale_int_length(int length) const;
  [[nodiscard]] static double scale_int_angle(int angle);
  [[nodiscard]] int scale_double_length(double length) const;
  [[nodiscard]] static int scale_double_angle(double angle);

  [[nodiscard]] double theta_as_double() const;
  [[nodiscard]] double x_as_double() const;
  [[nodiscard]] double y_as_double() const;

  [[nodiscard]] int dx(int i) const { return delta_x_.at(i); };
  [[nodiscard]] int dy(int i) const { return delta_y_.at(i); };
  [[nodiscard]] int dtheta(int i) const { return delta_theta_.at(i); };

private:
  WalkerID ID_;

  double x0_;
  double y0_;
  double theta0_;

  int step_fragment_;
  double stepsize_;

  int x_;
  int y_;
  int theta_;
  double std_dev_pos_;
  double std_dev_ang_;
  double I11est_;
  double I22est_;
  double I33est_;

  std::vector<double> X_;
  std::vector<double> Y_;
  std::vector<double> Theta_;

  std::vector<double> odom_pos_{0.0};
  std::vector<double> odom_ang_{0.0};

  double drx_;
  double dry_;
  double drtheta_;

  std::vector<double> Drx_;
  std::vector<double> Dry_;
  std::vector<double> Drtheta_;

  std::vector<Loopclosure> LC_{};

  RNGenerator rng;

  std::vector<int> delta_x_;
  std::vector<int> delta_y_;
  std::vector<int> delta_theta_;

  int random_turn();
  void simulate_steps(int dtheta);
  void dead_reckon(double odom_ang, double odom_pos);
  [[nodiscard]] std::tuple<int, int> position_change_in_global_frame() const;
};

}  // namespace kollagen
#endif /* !WALKER_H */
