#include <cmath>

#ifndef LCPARAMS_H
#define LCPARAMS_H

namespace kollagen
{

/**
 * \brief Loop Closure Parameters.
 * For intra-agent loop closures.
 */
struct LCParams {
  LCParams() = default;
  LCParams(double sigma_pos, double sigma_ang, double estimated_information_pos, double estimated_information_ang,
           double radius, double idemLCprob,
           double prob_scale)
      : sigma_pos(sigma_pos), sigma_ang(sigma_ang),
        I11est(estimated_information_pos > 0 ? estimated_information_pos : std::pow(sigma_pos, -2)),
        I22est(estimated_information_pos > 0 ? estimated_information_pos : std::pow(sigma_pos, -2)),
        I33est(estimated_information_ang > 0 ? estimated_information_ang : std::pow(sigma_pos, -2)), radius(radius),
        idemLCprob(idemLCprob), prob_scale(prob_scale){};
  /**
   * \brief Standard deviation [length unit]
   *
   * Standard deviation of the noise added to the x and y components of the loop closure.
  */
  double sigma_pos{0.01};
  /**
   * \brief Standard deviation [radians]
   *
   * Standard deviation of the noise added to the angular component of the loop closure.
  */
  double sigma_ang{0.001};
  /**
   * \brief Information matrix entry I11 [1/(length unit)^2]
   *
   * Estimated Fisher information matrix entry for x (first diagonal entry).
  */
  double I11est{std::pow(sigma_pos, -2)};
  /**
   * \brief Information matrix entry I22 [1/(length unit)^2]
   *
   * Estimated Fisher information matrix entry for y (second diagonal entry).
  */
  double I22est{std::pow(sigma_pos, -2)};
  /**
   * \brief Information matrix entry I33 [1/radians^2]
   *
   * Estimated Fisher information matrix entry for theta (third diagonal entry).
  */
  double I33est{std::pow(sigma_ang, -2)};
  /**
   * \brief Maximum loop closure distance [length unit]
   *
   * This sets the size of the circle used to check for loop closure
   * candidates, which means it can be considered as an upper bound on the
   * distance for the loop closures.
  */
  double radius{3.0};
  /**
   * \brief Idem loop closure probability (unitless)
   *
   * This sets the probability of loop closure when revisiting a previously
   * visited position.
  */
  double idemLCprob{0.5};
  /**
   * \brief Loop closure probability scaling (unitless)
   *
   * This scales the probability of loop closure. Scaling is done through
   * multiplication, and thus setting to 1.0 means not scaling.
  */
  double prob_scale{0.05};
};

}  // namespace kollagen
#endif /* !LCPARAMS_H */
