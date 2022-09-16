#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

#include "kollagen.h"

using std::chrono::high_resolution_clock;
using std::chrono::duration;
using std::chrono::milliseconds;

int main() {

  double radius = 3.0;
  double idemLCprob = 0.5;
  double prob_scale = 0.05;

  int seed1 = 9875;

  double sigma_pos = 0.023;
  double sigma_ang = sigma_pos / 10;
  double Idiag = std::pow(sigma_pos, -2);

  double sigma_pos_single = 0.023;
  double sigma_ang_single = sigma_pos / 10;
  double Idiag_single = std::pow(sigma_pos_single, -2);

  double sigma_pos_inter = 0.023;
  double sigma_ang_inter = sigma_pos_inter / 10;
  double Idiag_inter = std::pow(sigma_pos_inter, -2);

  auto lc_param =
      LCParams(sigma_pos_single, sigma_ang_single, Idiag_single,
               Idiag_single, radius, idemLCprob, prob_scale);

  auto agent_param =
      AgentParams(1.0, 4, sigma_pos, sigma_ang, Idiag, Idiag * 100, seed1 + 0);
  auto agent2_param =
      AgentParams(1.0, 4, sigma_pos, sigma_ang, Idiag, Idiag * 100, seed1 + 1);
  auto agent3_param =
      AgentParams(1.0, 4, sigma_pos, sigma_ang, Idiag, Idiag * 100, seed1 + 2);
  auto agent4_param =
      AgentParams(1.0, 4, sigma_pos, sigma_ang, Idiag, Idiag * 100, seed1 + 3);
  auto agent5_param =
      AgentParams(1.0, 4, sigma_pos, sigma_ang, Idiag, Idiag * 100, seed1 + 4);

  auto inter_lc_param =
      InterLCParams(sigma_pos_inter, sigma_ang_inter, Idiag_inter,
                    Idiag_inter, radius, 1.0, prob_scale, seed1 + 5);

  std::vector<AgentParams> agent_params{
    agent_param,
    agent2_param,
    agent3_param,
    agent4_param,
    agent5_param
  };


  std::vector<AgentParams> agents;
  for (int i = 0; i < 5; ++i) {
    std::cout << "i = " << i << '\n';
    std::vector<int> number_of_nodes{};
    std::vector<double> times{};

    agents.emplace_back(agent_params.at(i));
    for (int N = 1; N <= 2000000; N = N*2) {
      std::cout << "\tN = " << N << '\n';
      auto t1 = high_resolution_clock::now();
      DataGenerator data_gen = DataGenerator(
          {{agents}, lc_param, inter_lc_param, N}
      );
      data_gen.generate();
      auto t2 = high_resolution_clock::now();

      duration<double, std::milli> ms_double = t2 - t1;
      double time = ms_double.count();
      times.emplace_back(time);
      number_of_nodes.emplace_back(N);
    }
    save_binary("timing_x"+std::to_string(i+1)+".bin", number_of_nodes);
    save_binary("timing_y"+std::to_string(i+1)+".bin", times);
  }

  for (int N = 10; N <= 100000; N = N*10) {
    std::cout << "N = " << N << '\n';
    std::vector<AgentParams> agents;
    std::vector<int> number_of_agents{};
    std::vector<double> times{};

    for (int i = 1; i <= 1024; i = i*2) {
      std::cout << "\ti = " << i << '\n';
      agents.emplace_back(AgentParams(1.0, 1, sigma_pos, sigma_ang, Idiag, Idiag, seed1 + 1));
      auto t1 = high_resolution_clock::now();
      DataGenerator data_gen = DataGenerator(
          {{agents}, lc_param, inter_lc_param, N}
      );
      data_gen.generate();
      auto t2 = high_resolution_clock::now();

      duration<double, std::milli> ms_double = t2 - t1;
      double time = ms_double.count();
      times.emplace_back(time);
      number_of_agents.emplace_back(i+1);
    }
    save_binary("timing_agents_x"+std::to_string(N)+".bin", number_of_agents);
    save_binary("timing_agents_y"+std::to_string(N)+".bin", times);
  }

  for (int N = 10; N <= 1000; N = N*10) {
    std::cout << "N = " << N << '\n';
    std::vector<int> radius{};
    std::vector<double> times{};

    for (int i = 1; i <= 128; i = i*2) {
      std::cout << "\ti = " << i << '\n';
      lc_param.radius = i;
      inter_lc_param.radius = i;
      auto t1 = high_resolution_clock::now();
      DataGenerator data_gen = DataGenerator(
          {{agent_params}, lc_param, inter_lc_param, N}
      );
      data_gen.generate();
      auto t2 = high_resolution_clock::now();

      duration<double, std::milli> ms_double = t2 - t1;
      double time = ms_double.count();
      times.emplace_back(time);
      radius.emplace_back(i);
    }
    save_binary("timing_radius_x"+std::to_string(N)+".bin", radius);
    save_binary("timing_radius_y"+std::to_string(N)+".bin", times);
  }


  return EXIT_SUCCESS;
}


