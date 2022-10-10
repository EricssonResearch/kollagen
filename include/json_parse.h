#include "json.hpp"
#include "DataGeneratorParams.h"
#include <fstream>
#include <iostream>

using json = nlohmann::json;

inline json open_json(const std::string& filename){
  std::ifstream f(filename);
  json data = json::parse(f);
  return data;
}

inline std::vector<AgentParams> parse_agent_params(const json& data){
  std::vector<AgentParams> agents{};
  int N_agents = data.at("number_of_agents");
  double stepsize = data.at("agent_parameters").at("stepsize");
  int step_fragment = data.at("agent_parameters").at("step_fragment");
  bool allow_retrocession = data.at("agent_parameters").at("allow_retrocession");
  for (const auto& agent_data : data.at("agent_parameters").at("agent_specific")) {
    AgentParams params;
    params.stepsize = stepsize;
    params.step_fragment = step_fragment;
    params.allow_retrocession = allow_retrocession;
    params.sigma_pos = agent_data.at("std_dev_position");
    params.sigma_ang = agent_data.at("std_dev_angle");
    params.I11est = agent_data.at("estimated_information_position");
    params.I22est = agent_data.at("estimated_information_position");
    params.I33est = agent_data.at("estimated_information_angle");
    params.seed = agent_data.at("random_number_generator_seed");
    agents.emplace_back(params);
  }
  if (N_agents != static_cast<int>(agents.size())) {
    if(agents.size() != 1){
      std::cerr << "[ERROR] Number of entries in 'agent_specific' field of "
        "json file must either be equal to 'number_of_agents', or equal to 1."
        << std::endl;
      std::terminate();
    };
    for (int i = 0; i < N_agents - 1; ++i) {
      agents.emplace_back(agents.at(0));
    }
  }
  return agents;
}

inline std::vector<LCParams> parse_lc_params(const json& data){
  std::vector<LCParams> lc_params{};
  int N_agents = data.at("number_of_agents");
  for (const auto& agent_data : data.at("agent_parameters").at("agent_specific")) {
    LCParams params;
    params.sigma_pos = agent_data.at("std_dev_intra_loop_closure_position");
    params.sigma_ang = agent_data.at("std_dev_intra_loop_closure_angle");
    params.I11est = agent_data.at("estimated_information_intra_position");
    params.I22est = agent_data.at("estimated_information_intra_position");
    params.I33est = agent_data.at("estimated_information_intra_angle");
    params.radius = agent_data.at("intra_loop_closure_radius");
    params.idemLCprob = agent_data.at("same_position_intra_loop_closure_probability");
    params.prob_scale = agent_data.at("intra_loop_closure_probability_scaling");
    lc_params.emplace_back(params);
  }
  if (N_agents != static_cast<int>(lc_params.size())) {
    if(lc_params.size() != 1){
      std::cerr << "[ERROR] Number of entries in 'agent_specific' field of "
        "json file must either be equal to 'number_of_agents', or equal to 1."
        << std::endl;
      std::terminate();
    };
    for (int i = 0; i < N_agents - 1; ++i) {
      lc_params.emplace_back(lc_params.at(0));
    }
  }
  return lc_params;
}

inline InterLCParams parse_inter_lc_params(const json& data){
  InterLCParams inter_lc_params{};
  auto inter_lc_data = data.at("inter_agent_loop_closure_parameters");
  inter_lc_params.sigma_pos = inter_lc_data.at("std_dev_loop_closure_position");
  inter_lc_params.sigma_ang = inter_lc_data.at("std_dev_loop_closure_angle");
  inter_lc_params.I11est = inter_lc_data.at("estimated_information_position");
  inter_lc_params.I22est = inter_lc_data.at("estimated_information_position");
  inter_lc_params.I33est = inter_lc_data.at("estimated_information_angle");
  inter_lc_params.radius = inter_lc_data.at("loop_closure_radius");
  inter_lc_params.idemLCprob = inter_lc_data.at("same_position_loop_closure_probability");
  inter_lc_params.prob_scale = inter_lc_data.at("loop_closure_probability_scaling");
  inter_lc_params.seed = inter_lc_data.at("random_number_generator_seed");
  return inter_lc_params;
}

inline DataGeneratorParams get_params_from_json(const json& data){
  auto agent_params = parse_agent_params(data);
  auto lc_params = parse_lc_params(data);
  auto inter_lc_params = parse_inter_lc_params(data);
  int N_steps = data.at("number_of_steps");
  auto params = DataGeneratorParams(agent_params,lc_params,inter_lc_params,N_steps);
  params.align_center_of_masses = data.at("align");
  return params;
}

inline DataGeneratorParams get_params_from_json(const std::string& filename){
  return get_params_from_json(open_json(filename));
}
