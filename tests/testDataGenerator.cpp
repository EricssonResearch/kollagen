#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "kollagen.h"
#include "DataGeneratorAttorney.h"

class TestDataGenerator : public ::testing::Test {
protected:
  void SetUp() override {
    using ::testing::Return;
    walker_params1.sigma_ang = 0.0;
    walker_params1.sigma_pos = 0.0;
    walker_params1.seed = 1;

    walker_params2 = walker_params1;
    walker_params2.y = 1.0;
    walker_params2.seed = 2;

    walker_params3 = walker_params1;
    walker_params3.y = 2.0;
    walker_params3.seed = 3;

    walker_param_vec = {walker_params1, walker_params2, walker_params3};

    lc_params.idemLCprob = 1.0;
    lc_params.sigma_ang = 0.0;
    lc_params.sigma_pos = 0.0;

    lc_param_vec = {lc_params, lc_params, lc_params};

    inter_lc_params.idemLCprob = 1.0;
    inter_lc_params.radius = 1.0;
  }

  WalkerParams walker_params1{};
  WalkerParams walker_params2;
  WalkerParams walker_params3;

  std::vector<WalkerParams> walker_param_vec;

  LCParams lc_params;
  std::vector<LCParams> lc_param_vec;

  InterLCParams inter_lc_params;

  DataGenerator data_gen;
};

bool correct_walker_ids(const DataGenerator& data_gen){
  auto walkers = data_gen.Walkers();
  std::vector<int> walkers_IDs;
  std::vector<int> correct_IDs;
  for (int i = 0; i < static_cast<int>(walkers.size()); ++i) {
    correct_IDs.emplace_back(i);
    walkers_IDs.emplace_back(walkers.at(i).ID());
  }
  return std::is_permutation(correct_IDs.begin(), correct_IDs.begin(), walkers_IDs.begin());
}

TEST_F(TestDataGenerator, testConstructor) {
  correct_walker_ids(data_gen);
  ASSERT_EQ(data_gen.Walkers().size(), 0);
}

TEST_F(TestDataGenerator, testConstructor2) {
  DataGenerator data_gen2 = DataGeneratorAttorney::ConstructDataGenerator(walker_param_vec);
  correct_walker_ids(data_gen2);
  ASSERT_EQ(data_gen2.Walkers().size(), 3);
}

TEST_F(TestDataGenerator, testConstructor3) {
  auto a = DataGeneratorAttorney::ConstructDataGenerator(5);
  correct_walker_ids(a);
  ASSERT_EQ(a.Walkers().size(), 5);
}

TEST_F(TestDataGenerator, testConstructor4) {
  int number_of_steps = 100;
  DataGenerator a = DataGeneratorAttorney::ConstructDataGenerator(7, number_of_steps);
  correct_walker_ids(a);
  int step_fragment = a.Walkers().at(0).step_fragment();
  int expected_number_of_steps = number_of_steps * step_fragment + 1;
  ASSERT_EQ(a.Walkers().size(), 7);
  for (const auto& walker : a.Walkers()) {
    ASSERT_EQ(static_cast<int>(walker.size()), expected_number_of_steps);
  }
}

TEST_F(TestDataGenerator, testAgentParams) {
  auto agent_params = AgentParams(0.1, 2, 0.3, 0.4, 0.5, 0.6, 4729);
  auto dg_params = DataGeneratorParams(agent_params, lc_params, inter_lc_params);
  dg_params.align_center_of_masses = false;
  DataGenerator data_gen(dg_params);
  data_gen.generate();
  auto walker_params = data_gen.Walkers().at(0).Params();
  EXPECT_DOUBLE_EQ(walker_params.x, 0);
  EXPECT_DOUBLE_EQ(walker_params.y, 0);
  EXPECT_DOUBLE_EQ(walker_params.theta, 0);
  EXPECT_DOUBLE_EQ(walker_params.stepsize, agent_params.stepsize);
  EXPECT_DOUBLE_EQ(walker_params.sigma_pos, agent_params.sigma_pos);
  EXPECT_DOUBLE_EQ(walker_params.sigma_ang, agent_params.sigma_ang);
  EXPECT_DOUBLE_EQ(walker_params.I11est, agent_params.I11est);
  EXPECT_DOUBLE_EQ(walker_params.I22est, agent_params.I22est);
  EXPECT_DOUBLE_EQ(walker_params.I33est, agent_params.I33est);
  EXPECT_DOUBLE_EQ(walker_params.seed, agent_params.seed);
  EXPECT_EQ(walker_params.step_fragment, agent_params.step_fragment);
}

TEST_F(TestDataGenerator, testAdd) {
  correct_walker_ids(data_gen);
  DataGeneratorAttorney::add_walker(data_gen);
  ASSERT_EQ(data_gen.Walkers().size(), 1);
}

TEST_F(TestDataGenerator, testAdd2) {
  correct_walker_ids(data_gen);
  DataGeneratorAttorney::add_walker(data_gen, WalkerParams());
  ASSERT_EQ(data_gen.Walkers().size(), 1);
}

TEST_F(TestDataGenerator, testAdd3) {
  correct_walker_ids(data_gen);
  DataGeneratorAttorney::add_walker(data_gen, walker_param_vec);
  ASSERT_EQ(data_gen.Walkers().size(), 3);
}

TEST_F(TestDataGenerator, testAdd4) {
  correct_walker_ids(data_gen);
  DataGeneratorAttorney::add_walker(data_gen, walker_param_vec);
  ASSERT_EQ(data_gen.Walkers().size(), 3);
}

void sort_pairs(std::vector<std::array<int,2>>& pairs){
  using pair_array = std::array<int, 2>;
  std::sort(pairs.begin(), pairs.end(), [](const pair_array& lhs, const pair_array& rhs){ return lhs.front() < rhs.front();});
  std::stable_sort(pairs.begin(), pairs.end(), [](const pair_array& lhs, const pair_array& rhs){ return lhs.front() < rhs.front();});
}

void test_pairs(std::vector<std::array<int,2>>& correct, int N){
  auto data_gen = DataGeneratorAttorney::ConstructDataGenerator(N);
  auto pairs = DataGeneratorAttorney::get_all_pairs(data_gen);
  sort_pairs(pairs);
  sort_pairs(correct);
  EXPECT_EQ(pairs.size(), correct.size());
  for (int i = 0; i < static_cast<int>(pairs.size()); ++i) {
    EXPECT_EQ(pairs.at(i), correct.at(i));
  }
}

TEST_F(TestDataGenerator, testGetAllPairs2Walkers){
  std::vector<std::array<int,2>> correct{
    {0,1}
  };
  test_pairs(correct, 2);
}

TEST_F(TestDataGenerator, testGetAllPairs3Walkers){
  std::vector<std::array<int,2>> correct{
    {0,1},
    {0,2},
    {1,2},
  };
  test_pairs(correct,3);
}

TEST_F(TestDataGenerator, testSaveAndLoadBinary){
  DataGenerator a = DataGeneratorAttorney::ConstructDataGenerator(1, 100);
  auto walker = a.Walkers().front();
  a.save_poses_to_binary("walker");
  auto x1 = load_binary<double>("walker1_x.bin");
  auto y1 = load_binary<double>("walker1_y.bin");
  auto theta1 = load_binary<double>("walker1_theta.bin");
  EXPECT_EQ(x1, walker.X());
  EXPECT_EQ(y1, walker.Y());
  EXPECT_EQ(theta1, walker.Theta());
}

TEST_F(TestDataGenerator, testSaveMultiG2O){
  DataGeneratorAttorney::add_walker(data_gen, walker_param_vec);
  DataGeneratorAttorney::step(data_gen, 400);
  DataGeneratorAttorney::intra_agent_loopclose(data_gen, lc_param_vec);
  DataGeneratorAttorney::inter_agent_loopclose(data_gen, inter_lc_params);
  DataGeneratorAttorney::set_data_has_been_generated(data_gen, true);
  EXPECT_EQ(data_gen.save_multig2o("multig2o"), EXIT_SUCCESS);
}

TEST_F(TestDataGenerator, testSaveSingleG2O){
  for (auto& param : walker_param_vec) {
    param.step_fragment = 1;
  }
  inter_lc_params.prob_scale=1.0;
  inter_lc_params.radius=1.0;
  DataGeneratorAttorney::add_walker(data_gen, walker_param_vec);
  DataGeneratorAttorney::step(data_gen, 4);
  DataGeneratorAttorney::intra_agent_loopclose(data_gen, lc_param_vec);
  DataGeneratorAttorney::inter_agent_loopclose(data_gen, inter_lc_params);
  DataGeneratorAttorney::set_data_has_been_generated(data_gen, true);
  EXPECT_EQ(data_gen.save_singleg2o("single.g2o", false), EXIT_SUCCESS);
}

TEST_F(TestDataGenerator, testAlign){
  walker_param_vec.pop_back();
  walker_param_vec.at(1).x += 1.0;
  DataGenerator a = DataGeneratorAttorney::ConstructDataGenerator(walker_param_vec);
  DataGeneratorAttorney::step_forward(a);
  DataGeneratorAttorney::step_forward(a);
  DataGeneratorAttorney::step_right(a);
  DataGeneratorAttorney::step_left(a);

  DataGeneratorAttorney::align_center_of_masses(a);

  double x1 = a.Walkers().at(0).X().at(0);
  double x2 = a.Walkers().at(1).X().at(0);

  double y1 = a.Walkers().at(0).Y().at(0);
  double y2 = a.Walkers().at(1).Y().at(0);

  EXPECT_DOUBLE_EQ(x1, x2);
  EXPECT_DOUBLE_EQ(y1, y2);
}

int max_seed_from_param_vec(std::vector<WalkerParams> param_vec){
  DataGenerator a = DataGeneratorAttorney::ConstructDataGenerator(param_vec);
  return DataGeneratorAttorney::get_max_seed(a);
}

TEST_F(TestDataGenerator, testGetMaxSeed){
  EXPECT_EQ(max_seed_from_param_vec(walker_param_vec), 3);
  walker_param_vec.pop_back();

  EXPECT_EQ(max_seed_from_param_vec(walker_param_vec), 2);
  walker_param_vec.pop_back();

  EXPECT_EQ(max_seed_from_param_vec(walker_param_vec), 1);

  walker_param_vec.at(0).seed = 2468;
  EXPECT_EQ(max_seed_from_param_vec(walker_param_vec), 2468);
}

TEST_F(TestDataGenerator, testSeedAlreadyUsed){
  walker_param_vec.at(0).seed = 2468;
  DataGenerator a = DataGeneratorAttorney::ConstructDataGenerator(walker_param_vec);
  EXPECT_TRUE(DataGeneratorAttorney::seed_already_used(a, 2468));
  EXPECT_TRUE(DataGeneratorAttorney::seed_already_used(a, 2));
  EXPECT_TRUE(DataGeneratorAttorney::seed_already_used(a, 3));
}

bool all_walkers_have_same_stepsize(const DataGenerator& a){
  auto walkers = a.Walkers();
  auto stepsize = walkers.at(0).stepsize();
  return std::ranges::all_of(walkers.cbegin(), walkers.cend(), [&stepsize](const Walker& walker){ return walker.stepsize() == stepsize; });
}

bool all_walkers_have_same_step_fragment(const DataGenerator& a){
  auto walkers = a.Walkers();
  auto step_fragment = walkers.at(0).step_fragment();
  return std::ranges::all_of(walkers.cbegin(), walkers.cend(), [&step_fragment](const Walker& walker){ return walker.step_fragment() == step_fragment; });
}

TEST_F(TestDataGenerator, testAddWalker){
  walker_param_vec.at(0).stepsize = 1.0;
  walker_param_vec.at(1).stepsize = 2.5;
  walker_param_vec.at(2).stepsize = 1.0;
  walker_param_vec.at(2).step_fragment = 9;
  DataGenerator a = DataGeneratorAttorney::ConstructDataGenerator(walker_param_vec);
  EXPECT_TRUE(all_walkers_have_same_stepsize(a));
  EXPECT_TRUE(all_walkers_have_same_step_fragment(a));
}
