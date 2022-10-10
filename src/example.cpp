#include <cmath>
#include <iostream>
#include <vector>

#include "kollagen.h"

int main() {

  int N_steps = 1200;
  double radius = 3.0;
  double idemLCprob = 0.5;
  double prob_scale = 0.05;

  int seed1 = 1406;
  int seed2 = 1302;

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
                    Idiag_inter, radius, 1.0, prob_scale, seed1 + seed2);

  DataGenerator data_gen =
      DataGenerator({{agent_param, agent2_param, agent3_param, agent4_param,
                      agent5_param},
                     lc_param,
                     inter_lc_param,
                     N_steps});

  data_gen.generate();

  data_gen.save_multig2o("Example4800");
  data_gen.save_singleg2o("Example4800.g2o", false);
  data_gen.save_singleTUM("Example4800_GT.tum");
  data_gen.save_singleTUM("Example4800.tum", false);

  std::cout << data_gen.LCs().size() << " inter agent LCs!" << std::endl;

  std::cout << "Saving files x.bin and y.bin... ";
  for (size_t i = 0; i < data_gen.Walkers().size(); ++i) {
    std::string label = std::to_string(i + 1);
    save_binary("x" + label + ".bin", data_gen.Walkers().at(i).X());
    save_binary("y" + label + ".bin", data_gen.Walkers().at(i).Y());
  }
  std::cout << "Done!" << std::endl;

  return EXIT_SUCCESS;
}
