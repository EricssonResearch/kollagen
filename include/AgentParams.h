#include "WalkerParams.h"

#ifndef AGENTPARAMS_H
#define AGENTPARAMS_H

/**
 * \brief Agent Parameters.
 * The agent parameters controls the behavior of the agents.
 */
struct AgentParams : public WalkerParams {
  AgentParams() = default;
  /**
   * \brief Constructor.
   *
   * \param stepsize Step size [length unit].
   *
   * \param step_fragment Divides the steps in "sub"-steps (unitless).  The
   * number of steps taken before (possibly) turning. By setting this to a
   * factor greater than 1, the actual number of steps are increased by this
   * factor, and the actual step size is decreased by the same factor.
   *
   * \param sigma_pos Standard deviation [length unit]. Standard deviation of
   * the noise added to the \f$x\f$ and \f$y\f$ components of the odometry.
   *
   * \param sigma_ang Standard deviation [radians] Standard deviation of the
   * noise added to the angular component of the odometry.
   *
   * \param estimated_information_pos Information matrix entries for position
   * [1/(length unit)^2]. Estimated Fisher information matrix entry for
   * \f$x\f$ and \f$y\f$ (first and second diagonal entries).
   *
   * \param estimated_information_ang Information matrix entry for heading
   * angle [1/radians^2]. Estimated Fisher information matrix entry for \f$\theta\f$
   * (third diagonal entry).
   *
   * \param seed Random Number Generator seed (unitless). Seed used for the
   * random walk, and the intra-agent loop closure.
   *
   * \param allow_retrocession If set to true, the agent is able to turn 180
   * degrees, and thus retrocede.
   */
  AgentParams(double stepsize, int step_fragment, double sigma_pos,
              double sigma_ang, double estimated_information_pos,
              double estimated_information_ang, int seed,
              bool allow_retrocession = true)
      : WalkerParams(0, 0, 0, stepsize, sigma_pos, sigma_ang, step_fragment,
                     seed, 0, estimated_information_pos,
                     estimated_information_pos, estimated_information_ang,
                     allow_retrocession){};
};

#endif /* !AGENTPARAMS_H */
