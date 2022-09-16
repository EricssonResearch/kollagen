#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "kollagen.h"
#include "json_parse.h"

class TestJSONParse : public ::testing::Test {
protected:
  void SetUp() override {
    test_data = open_json("test_data/test.json");
  }
  json test_data;
};

TEST_F(TestJSONParse, testParseFromFile){
  EXPECT_EQ(test_data["number_of_agents"], 2);
  EXPECT_EQ(test_data["agent_parameters"]["stepsize"], 1.5);
  EXPECT_EQ(test_data["agent_parameters"]["step_fragment"], 2);
  EXPECT_EQ(test_data["agent_parameters"]["agent_specific"].size(), 2);
  EXPECT_EQ(test_data["agent_parameters"]["agent_specific"].at(0)["std_dev_position"], 1);
  EXPECT_EQ(test_data["agent_parameters"]["agent_specific"].at(0)["std_dev_angle"], 2);
  EXPECT_EQ(test_data["agent_parameters"]["agent_specific"].at(0)["estimated_information_position"], 3);
  EXPECT_EQ(test_data["agent_parameters"]["agent_specific"].at(0)["estimated_information_angle"], 4);
  EXPECT_EQ(test_data["agent_parameters"]["agent_specific"].at(0)["std_dev_intra_loop_closure_position"], 5);
  EXPECT_EQ(test_data["agent_parameters"]["agent_specific"].at(0)["std_dev_intra_loop_closure_angle"], 6);
  EXPECT_EQ(test_data["agent_parameters"]["agent_specific"].at(0)["estimated_information_intra_position"], 7);
  EXPECT_EQ(test_data["agent_parameters"]["agent_specific"].at(0)["estimated_information_intra_angle"], 8);
  EXPECT_EQ(test_data["agent_parameters"]["agent_specific"].at(0)["intra_loop_closure_radius"], 9);
  EXPECT_EQ(test_data["agent_parameters"]["agent_specific"].at(0)["agent_random_number_generator_seed"], 10);
  EXPECT_EQ(test_data["agent_parameters"]["agent_specific"].at(0)["same_position_intra_loop_closure_probability"], 11);
  EXPECT_EQ(test_data["agent_parameters"]["agent_specific"].at(1)["std_dev_position"], 12);
  EXPECT_EQ(test_data["agent_parameters"]["agent_specific"].at(1)["std_dev_angle"], 13);
  EXPECT_EQ(test_data["agent_parameters"]["agent_specific"].at(1)["estimated_information_position"], 14);
  EXPECT_EQ(test_data["agent_parameters"]["agent_specific"].at(1)["estimated_information_angle"], 15);
  EXPECT_EQ(test_data["agent_parameters"]["agent_specific"].at(1)["std_dev_intra_loop_closure_position"], 16);
  EXPECT_EQ(test_data["agent_parameters"]["agent_specific"].at(1)["std_dev_intra_loop_closure_angle"], 17);
  EXPECT_EQ(test_data["agent_parameters"]["agent_specific"].at(1)["estimated_information_intra_position"], 18);
  EXPECT_EQ(test_data["agent_parameters"]["agent_specific"].at(1)["estimated_information_intra_angle"], 19);
  EXPECT_EQ(test_data["agent_parameters"]["agent_specific"].at(1)["intra_loop_closure_radius"], 20);
  EXPECT_EQ(test_data["agent_parameters"]["agent_specific"].at(1)["agent_random_number_generator_seed"], 21);
  EXPECT_EQ(test_data["agent_parameters"]["agent_specific"].at(1)["same_position_intra_loop_closure_probability"], 22);
  EXPECT_EQ(test_data["inter_agent_loop_closure_parameters"]["same_position_inter_loop_closure_probability"], 23);
  EXPECT_EQ(test_data["inter_agent_loop_closure_parameters"]["random_number_generator_seed"], 24);
}
