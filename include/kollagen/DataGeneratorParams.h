#include <cassert>
#include <utility>
#include <vector>

#include "AgentParams.h"
#include "InterLCParams.h"
#include "LCParams.h"

#ifndef DATAGENERATORPARAMS_H
#define DATAGENERATORPARAMS_H

namespace kollagen {

/**
 * \brief Data Generator Parameters.
 *
 */
struct DataGeneratorParams {
  DataGeneratorParams()
      : agent_params({AgentParams()}), lc_params({LCParams()}){};
  DataGeneratorParams(AgentParams Wp, LCParams LCp, InterLCParams ILCp,
                      bool exact_information_matrix = false)
      : agent_params({Wp}), lc_params({LCp}), inter_lc_params(ILCp),
        exact_information_matrix(exact_information_matrix){};

  DataGeneratorParams(std::vector<AgentParams> Wp, std::vector<LCParams> LCp,
                      InterLCParams ILCp, int N_steps,
                      bool exact_information_matrix = false)
      : agent_params(std::move(Wp)), lc_params(std::move(LCp)),
        inter_lc_params(ILCp), N_agents(static_cast<int>(agent_params.size())),
        N_steps(N_steps), exact_information_matrix(exact_information_matrix) {

    assert(agent_params.size() == N_agents);
    assert(lc_params.size() == N_agents);
  };

  DataGeneratorParams(std::vector<AgentParams> Wp, LCParams LCp,
                      InterLCParams ILCp, int N_steps, bool
                      exact_information_matrix = false)
      : agent_params(std::move(Wp)), inter_lc_params(ILCp),
        N_agents(static_cast<int>(agent_params.size())), N_steps(N_steps),
        exact_information_matrix(exact_information_matrix) {
    lc_params.resize(N_agents);
    std::fill_n(lc_params.begin(), N_agents, LCp);
    assert(agent_params.size() == N_agents);
    assert(lc_params.size() == N_agents);
  };

  DataGeneratorParams(AgentParams Wp, std::vector<LCParams> LCp,
                      InterLCParams ILCp, int N_steps,
                      bool exact_information_matrix = false)
      : lc_params(std::move(LCp)), inter_lc_params(ILCp),
        N_agents(static_cast<int>(agent_params.size())), N_steps(N_steps),
        exact_information_matrix(exact_information_matrix) {
    agent_params.resize(N_agents);
    std::fill_n(agent_params.begin(), N_agents, Wp);
    assert(agent_params.size() == N_agents);
    assert(lc_params.size() == N_agents);
  };

  /**
   * \brief Agent parameters
  */
  std::vector<AgentParams> agent_params{};
  /**
   * \brief Intra-agent loop-closure parameters.
  */
  std::vector<LCParams> lc_params{};
  /**
   * \brief Inter-agent loop-closure parameters.
  */
  InterLCParams inter_lc_params{};
  /**
   * \brief Number of agents
  */
  int N_agents{1};
  /**
   * \brief Number of steps for each agent
  */
  int N_steps{100};
  /**
   * \brief Align the agents' centers of mass
  */
  bool align_center_of_masses{true};
  /**
   * \brief Use exact information matrix
  */
  bool exact_information_matrix{false};
};

} // namespace kollagen
#endif /* !DATAGENERATORPARAMS_H */
