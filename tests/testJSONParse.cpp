#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "kollagen.h"

class TestJSONParse : public ::testing::Test {
protected:
  void SetUp() override {
    test_data = kollagen::open_json("test_data/test.json");
    test_data2 = kollagen::open_json("test_data/test2.json");
  }
  nlohmann::json test_data;
  nlohmann::json test_data2;
};

TEST_F(TestJSONParse, testParseAgentParams){
  auto test = kollagen::parse_agent_params(test_data);
  EXPECT_EQ(test_data.at("number_of_agents"), test.size());
  for (int i = 0; const auto& agent : test) {
    auto agent_data = test_data.at("agent_parameters").at("agent_specific").at(i);
    EXPECT_EQ(test_data.at("agent_parameters").at("stepsize"), agent.stepsize);
    EXPECT_EQ(test_data.at("agent_parameters").at("step_fragment"), agent.step_fragment);
    EXPECT_EQ(test_data.at("agent_parameters").at("allow_retrocession"), agent.allow_retrocession);
    EXPECT_EQ(agent_data.at("std_dev_position"), agent.sigma_pos);
    EXPECT_EQ(agent_data.at("std_dev_angle"), agent.sigma_ang);
    EXPECT_EQ(agent_data.at("estimated_information_position"), agent.I11est);
    EXPECT_EQ(agent_data.at("estimated_information_position"), agent.I22est);
    EXPECT_EQ(agent_data.at("estimated_information_angle"), agent.I33est);
    EXPECT_EQ(agent_data.at("random_number_generator_seed"), agent.seed);
    ++i;
  }
}

TEST_F(TestJSONParse, testParseAgentParams2){
  auto test = kollagen::parse_agent_params(test_data2);
  EXPECT_EQ(test_data2.at("number_of_agents"), test.size());
  for (const auto& agent : test) {
    auto agent_data = test_data2.at("agent_parameters").at("agent_specific").at(0);
    EXPECT_EQ(test_data2.at("agent_parameters").at("stepsize"), agent.stepsize);
    EXPECT_EQ(test_data2.at("agent_parameters").at("step_fragment"), agent.step_fragment);
    EXPECT_EQ(test_data2.at("agent_parameters").at("allow_retrocession"), agent.allow_retrocession);
    EXPECT_EQ(agent_data.at("std_dev_position"), agent.sigma_pos);
    EXPECT_EQ(agent_data.at("std_dev_angle"), agent.sigma_ang);
    EXPECT_EQ(agent_data.at("estimated_information_position"), agent.I11est);
    EXPECT_EQ(agent_data.at("estimated_information_position"), agent.I22est);
    EXPECT_EQ(agent_data.at("estimated_information_angle"), agent.I33est);
    EXPECT_EQ(agent_data.at("random_number_generator_seed"), agent.seed);
  }
}

TEST_F(TestJSONParse, testParseLCParams){
  auto test = kollagen::parse_lc_params(test_data);
  for (int i = 0; const auto& lc : test) {
    auto lc_data = test_data.at("agent_parameters").at("agent_specific").at(i);
    EXPECT_EQ(lc_data.at("std_dev_intra_loop_closure_position"), lc.sigma_pos);
    EXPECT_EQ(lc_data.at("std_dev_intra_loop_closure_angle"), lc.sigma_ang);
    EXPECT_EQ(lc_data.at("estimated_information_intra_position"), lc.I11est);
    EXPECT_EQ(lc_data.at("estimated_information_intra_position"), lc.I22est);
    EXPECT_EQ(lc_data.at("estimated_information_intra_angle"), lc.I33est);
    EXPECT_EQ(lc_data.at("intra_loop_closure_radius"), lc.radius);
    EXPECT_EQ(lc_data.at("same_position_intra_loop_closure_probability"), lc.idemLCprob);
    EXPECT_EQ(lc_data.at("intra_loop_closure_probability_scaling"), lc.prob_scale);
    ++i;
  }
}

TEST_F(TestJSONParse, testParseLCParams2){
  auto test = kollagen::parse_lc_params(test_data2);
  for (const auto& lc : test) {
    auto lc_data = test_data.at("agent_parameters").at("agent_specific").at(0);
    EXPECT_EQ(lc_data.at("std_dev_intra_loop_closure_position"), lc.sigma_pos);
    EXPECT_EQ(lc_data.at("std_dev_intra_loop_closure_angle"), lc.sigma_ang);
    EXPECT_EQ(lc_data.at("estimated_information_intra_position"), lc.I11est);
    EXPECT_EQ(lc_data.at("estimated_information_intra_position"), lc.I22est);
    EXPECT_EQ(lc_data.at("estimated_information_intra_angle"), lc.I33est);
    EXPECT_EQ(lc_data.at("intra_loop_closure_radius"), lc.radius);
    EXPECT_EQ(lc_data.at("same_position_intra_loop_closure_probability"), lc.idemLCprob);
    EXPECT_EQ(lc_data.at("intra_loop_closure_probability_scaling"), lc.prob_scale);
  }
}

TEST_F(TestJSONParse, testParseInterLCParams){
  auto test = kollagen::parse_inter_lc_params(test_data);
  auto lc_data = test_data.at("inter_agent_loop_closure_parameters");
  EXPECT_EQ(lc_data.at("std_dev_loop_closure_position"), test.sigma_pos);
  EXPECT_EQ(lc_data.at("std_dev_loop_closure_angle"), test.sigma_ang);
  EXPECT_EQ(lc_data.at("estimated_information_position"), test.I11est);
  EXPECT_EQ(lc_data.at("estimated_information_position"), test.I22est);
  EXPECT_EQ(lc_data.at("estimated_information_angle"), test.I33est);
  EXPECT_EQ(lc_data.at("loop_closure_radius"), test.radius);
  EXPECT_EQ(lc_data.at("same_position_loop_closure_probability"), test.idemLCprob);
  EXPECT_EQ(lc_data.at("loop_closure_probability_scaling"), test.prob_scale);
  EXPECT_EQ(lc_data.at("random_number_generator_seed"), test.seed);
}

TEST_F(TestJSONParse, testGetParamsFromJSON){
  auto test = kollagen::get_params_from_json(test_data);
  EXPECT_EQ(test_data.at("align"), test.align_center_of_masses);
  EXPECT_EQ(test_data.at("number_of_steps"), test.N_steps);
  EXPECT_EQ(test_data.at("number_of_agents"), test.N_agents);
  EXPECT_EQ(test_data.at("exact_information_matrix"), test.exact_information_matrix);
}
