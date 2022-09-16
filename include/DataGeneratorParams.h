#include <cassert>
#include <utility>
#include <vector>

#include "InterLCParams.h"
#include "LCParams.h"
#include "AgentParams.h"

#ifndef DATAGENERATORPARAMS_H
#define DATAGENERATORPARAMS_H

struct DataGeneratorParams {
  DataGeneratorParams()
      : agent_params({AgentParams()}), lc_params({LCParams()}){};
  DataGeneratorParams(AgentParams Wp, LCParams LCp, InterLCParams ILCp)
      : agent_params({Wp}), lc_params({LCp}), inter_lc_params(ILCp){};

  DataGeneratorParams(std::vector<AgentParams> Wp, std::vector<LCParams> LCp,
                      InterLCParams ILCp, int N_steps)
      : agent_params(std::move(Wp)), lc_params(std::move(LCp)),
        inter_lc_params(ILCp), N_agents(static_cast<int>(agent_params.size())), N_steps(N_steps){
    
    assert(agent_params.size() == N_agents);
    assert(lc_params.size() == N_agents);
  };

  DataGeneratorParams(std::vector<AgentParams> Wp, LCParams LCp,
                      InterLCParams ILCp, int N_steps)
      : agent_params(std::move(Wp)),
        inter_lc_params(ILCp), N_agents(static_cast<int>(agent_params.size())), N_steps(N_steps){
    lc_params.resize(N_agents);
    std::fill_n(lc_params.begin(), N_agents, LCp);
    assert(agent_params.size() == N_agents);
    assert(lc_params.size() == N_agents);
  };

  DataGeneratorParams(AgentParams Wp, std::vector<LCParams> LCp,
                      InterLCParams ILCp, int N_steps)
      : lc_params(std::move(LCp)),
        inter_lc_params(ILCp), N_agents(static_cast<int>(agent_params.size())), N_steps(N_steps){
    agent_params.resize(N_agents);
    std::fill_n(agent_params.begin(), N_agents, Wp);
    assert(agent_params.size() == N_agents);
    assert(lc_params.size() == N_agents);
  };

  std::vector<AgentParams> agent_params{};
  std::vector<LCParams> lc_params{};
  InterLCParams inter_lc_params{};
  int N_agents{1};
  int N_steps{100};
  bool align_center_of_masses{true};
};

#endif /* !DATAGENERATORPARAMS_H */
