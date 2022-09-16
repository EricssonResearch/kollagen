#include "LCParams.h"
#ifndef INTERLCPARAMS_H
#define INTERLCPARAMS_H

/**
 * \brief Loop Closure Parameters.
 * For inter-agent loop closures.
 */
struct InterLCParams : public LCParams {
  InterLCParams() = default;
  InterLCParams(double sigma_pos, double sigma_ang,
                double estimated_information_pos,
                double estimated_information_ang, double radius,
                double idemLCprob, double prob_scale, int seed)
      : LCParams(sigma_pos, sigma_ang, estimated_information_pos,
                 estimated_information_ang, radius,
                 idemLCprob, prob_scale),
        seed(seed){};
  /**
   * \brief Random Number Generator seed (unitless)
   *
   * Seed used for inter-agent loop closures.
   */
  int seed{1234};
};

#endif /* !INTERLCPARAMS_H */
