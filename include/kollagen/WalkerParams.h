#include <cmath>

#ifndef WALKERPARAMS_H
#define WALKERPARAMS_H

namespace kollagen
{

struct WalkerParams {
  WalkerParams() = default;
  WalkerParams(double x, double y, double theta, double stepsize,
               double sigma_pos, double sigma_ang, int step_fragment, int seed,
               int ID, double I11est, double I22est, double I33est,
               bool allow_retrocession = true)
      : x(x), y(y), theta(theta), stepsize(stepsize), sigma_pos(sigma_pos),
        sigma_ang(sigma_ang), step_fragment(step_fragment), seed(seed), ID(ID),
        I11est(I11est), I22est(I22est), I33est(I33est),
        allow_retrocession(allow_retrocession){};
  /// Initial x-position [length unit]
  double x{0};
  /// Initial y-position [length unit]
  double y{0};
  /// Initial heading angle [radians]
  double theta{0};
  double stepsize{1.0};
  double sigma_pos{0.01};
  double sigma_ang{0.001};
  int step_fragment{4};
  int seed{1234};
  /// ID (unitless)
  int ID{1};
  double I11est{std::pow(sigma_pos, -2)};
  double I22est{std::pow(sigma_pos, -2)};
  double I33est{std::pow(sigma_ang, -2)};
  bool allow_retrocession{true};

  inline bool operator==(const WalkerParams& other) const{
    return x == other.x &&
           y == other.y &&
           theta == other.theta &&
           stepsize == other.stepsize &&
           sigma_pos == other.sigma_pos &&
           sigma_ang == other.sigma_ang &&
           step_fragment == other.step_fragment &&
           seed == other.seed &&
           ID == other.ID &&
           I11est == other.I11est &&
           I22est == other.I22est &&
           I33est == other.I33est &&
           allow_retrocession == other.allow_retrocession;
  }
};

}  // namespace kollagen
#endif /* !WALKERPARAMS_H */
