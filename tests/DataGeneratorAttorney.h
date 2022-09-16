#include "DataGenerator.h"

#ifndef DATAGENERATORATTORNEY_H
#define DATAGENERATORATTORNEY_H

class DataGeneratorAttorney
{
public:
  static void step_forward(DataGenerator& a){
    a.step_forward();
  }
  static void step_right(DataGenerator& a){
    a.step_right();
  }
  static void step_left(DataGenerator& a){
    a.step_left();
  }
  static void align_center_of_masses(DataGenerator& a){
    a.align_center_of_masses();
  }
  static std::vector<int> get_walker_seeds(DataGenerator& a){
    return a.get_walker_seeds();
  }
  static int get_max_seed(DataGenerator& a){
    return a.get_max_seed();
  }
  static bool seed_already_used(DataGenerator& a, int seed){
    return a.seed_already_used(seed);
  }
  static bool different_stepsize(DataGenerator& a, double stepsize){
    return a.different_stepsize(stepsize);
  }
  static bool different_step_fragment(DataGenerator& a, int step_fragment){
    return a.different_step_fragment(step_fragment);
  }
  static void step(DataGenerator& a, int number_of_steps){
    a.step(number_of_steps);
  }
  static void step(DataGenerator& a){
    a.step();
  }
  static void intra_agent_loopclose(DataGenerator& a, const LCParams& params){
    a.intra_agent_loopclose(params);
  }
  static void intra_agent_loopclose(DataGenerator& a, const std::vector<LCParams>& params){
    a.intra_agent_loopclose(params);
  }
  static void inter_agent_loopclose(DataGenerator& a, const InterLCParams& params){
    a.inter_agent_loopclose(params);
  }
  static std::vector<std::array<int, 2>> get_all_pairs(DataGenerator& a) {
    return a.get_all_pairs();
  }

  static void add_walker(DataGenerator& a){
    a.add_walker();
  }
  static void add_walker(DataGenerator& a, WalkerParams params){
    a.add_walker(params);
  }
  static void add_walker(DataGenerator& a, const std::vector<WalkerParams>& params){
    a.add_walker(params);
  }
  static void set_data_has_been_generated(DataGenerator& a, bool value){
    a.data_has_been_generated_ = value;
  };
  static void set_paramaters_have_been_set(DataGenerator& a, bool value){
    a.parameters_have_been_set_ = value;
  };

  static DataGenerator ConstructDataGenerator(int number_of_walkers);
  static DataGenerator ConstructDataGenerator(const std::vector<WalkerParams>& params);
  static DataGenerator ConstructDataGenerator(int number_of_walkers, int number_of_steps);

};

inline DataGenerator DataGeneratorAttorney::ConstructDataGenerator(int number_of_walkers){
  DataGenerator data_gen(number_of_walkers);
  return data_gen;
}

inline DataGenerator DataGeneratorAttorney::ConstructDataGenerator(const std::vector<WalkerParams>& params){
  DataGenerator data_gen(params);
  return data_gen;
}

inline DataGenerator DataGeneratorAttorney::ConstructDataGenerator(int number_of_walkers, int number_of_steps){
  DataGenerator data_gen(number_of_walkers, number_of_steps);
  return data_gen;
}

#endif /* !DATAGENERATORATTORNEY_H */
