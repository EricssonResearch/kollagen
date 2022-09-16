#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "kollagen.h"

class MockWalker : public Walker {
public:
  virtual ~MockWalker() = default;
  using Walker::Walker;
  MOCK_METHOD0(draw_uniform, double());
  MOCK_METHOD0(draw_normal, double());
};

class MockWalkerTest : public ::testing::Test {
protected:
  void SetUp() override {
    using ::testing::Return;

    ON_CALL(walker, draw_uniform).WillByDefault(Return(0.0));
    ON_CALL(walker, draw_normal).WillByDefault(Return(0.0));
    EXPECT_CALL(walker, draw_uniform).WillRepeatedly(Return(0.0));
    EXPECT_CALL(walker, draw_normal).WillRepeatedly(Return(0.0));

    ON_CALL(walker2, draw_uniform).WillByDefault(Return(0.0));
    ON_CALL(walker2, draw_normal).WillByDefault(Return(0.0));
    EXPECT_CALL(walker2, draw_uniform).WillRepeatedly(Return(0.0));
    EXPECT_CALL(walker2, draw_normal).WillRepeatedly(Return(0.0));

    ON_CALL(walker3, draw_uniform).WillByDefault(Return(0.0));
    ON_CALL(walker3, draw_normal).WillByDefault(Return(0.0));
    EXPECT_CALL(walker3, draw_uniform).WillRepeatedly(Return(0.0));
    EXPECT_CALL(walker3, draw_normal).WillRepeatedly(Return(0.0));

    walker.step_forward();
    walker.step_right();
    walker.step_left();
    walker.step_left();
    walker.step_left();
    walker.step_right();
    walker.step_forward();
    walker.step_forward();

    walker2.step_forward();
    walker2.step_forward();
    walker2.step_forward();
    walker2.step_forward();

    params.idemLCprob = 1.0;
    params.radius = 2.0;

    inter_params.idemLCprob = 1.0;
    inter_params.radius = 2.0;
    inter_params.sigma_pos = 0.0;
    inter_params.sigma_ang = 0.0;
  }
  WalkerParams walker_params1{0, 0, 0, 1.0, 0.1, 0.1, 1, 0, 1, 1.0, 1.0, 1.0};
  WalkerParams walker_params2{-1, 2, 0, 1.0, 0.1, 0.1, 1, 1, 2, 1.0, 1.0, 1.0};
  WalkerParams walker_params3{0, 0, 0, 1.0, 0.1, 0.1, 1, 2, 1, 1.0, 1.0, 1.0};
  WalkerParams walker_params4{0.2, -0.2, 0.5*M_PI, 0.2, 0.1, 0.1, 2, 2, 1, 1.0, 1.0, 1.0};

  MockWalker walker =
      MockWalker(walker_params1);
  MockWalker walker2 =
      MockWalker(walker_params2);

  MockWalker walker3 =
      MockWalker(walker_params3);

  MockWalker walker_4 =
      MockWalker(walker_params4);

  LCParams params = LCParams();
  InterLCParams inter_params = InterLCParams();
};

TEST(TestWalker, testNormalizeAngle) {
  std::vector<std::vector<double>> equalities = {
      {0.0, 0.0},      {-M_PI, -M_PI},    {M_PI, -M_PI},
      {-1*0.5*M_PI,-0.5*M_PI},
      { 1*0.5*M_PI, 0.5*M_PI},
      {-3*0.5*M_PI, 0.5*M_PI},
      { 3*0.5*M_PI,-0.5*M_PI},
      {2 * M_PI, 0.0}, {3 * M_PI, -M_PI}, {4 * M_PI, 0.0}};
  for (const auto &equality : equalities) {
    EXPECT_DOUBLE_EQ(normalize_angle(equality.at(0)), equality.at(1));
  }
}

TEST_F(MockWalkerTest, testParams){
  EXPECT_TRUE(walker_params1 == Walker(walker_params1).Params());
  EXPECT_TRUE(walker_params2 == Walker(walker_params2).Params());
  EXPECT_TRUE(walker_params3 == Walker(walker_params3).Params());
  EXPECT_TRUE(walker_params4 == Walker(walker_params4).Params());

  EXPECT_TRUE(walker_params1 != Walker(walker_params2).Params());
  EXPECT_TRUE(walker_params1 != Walker(walker_params3).Params());
  EXPECT_TRUE(walker_params1 != Walker(walker_params4).Params());
}


TEST_F(MockWalkerTest, testParamsEqualOperator){
  EXPECT_TRUE(walker_params1 == walker_params1);
  EXPECT_TRUE(walker_params2 == walker_params2);
  EXPECT_TRUE(walker_params3 == walker_params3);
  EXPECT_TRUE(walker_params4 == walker_params4);

  auto temp = walker_params4;
  walker_params4.x = 3.0;
  EXPECT_FALSE(walker_params4 == temp);

  EXPECT_TRUE(walker_params1 != walker_params2);
  EXPECT_TRUE(walker_params1 != walker_params3);
  EXPECT_TRUE(walker_params1 != walker_params4);
}

TEST_F(MockWalkerTest, testGroundTruth) {
  std::vector<double> expected_x{0, 1, 1, 2, 2, 1, 1, 1, 1};
  std::vector<double> expected_y{0, 0, -1, -1, 0, 0, 1, 2, 3};
  std::vector<double> expected_theta{
      0,    0,          -0.5 * M_PI, 0,          0.5 * M_PI,
     -M_PI, 0.5 * M_PI, 0.5 * M_PI,  0.5 * M_PI,
  };

  for (int i = 0; i < static_cast<int>(walker.size()); ++i) {
    EXPECT_NEAR(walker.X().at(i), expected_x.at(i), 1e-15);
    EXPECT_NEAR(walker.Y().at(i), expected_y.at(i), 1e-15);
    EXPECT_NEAR(walker.Theta().at(i), expected_theta.at(i), 1e-15);
  }
};

TEST_F(MockWalkerTest, testGroundTruth2) {
  std::vector<double> expected_x{-1, 0, 1, 2, 3};
  std::vector<double> expected_y{2, 2, 2, 2, 2};
  std::vector<double> expected_theta{0, 0, 0, 0, 0};
  std::cout << walker2.size() << std::endl;

  for (int i = 0; i < static_cast<int>(walker2.size()); ++i) {
    EXPECT_NEAR(walker2.X().at(i), expected_x.at(i), 1e-15);
    EXPECT_NEAR(walker2.Y().at(i), expected_y.at(i), 1e-15);
    EXPECT_NEAR(walker2.Theta().at(i), expected_theta.at(i), 1e-15);
  }
};

TEST_F(MockWalkerTest, testDeadReckoning) {
  std::vector<double> expected_x{0, 1, 1, 2, 2, 1, 1, 1, 1};
  std::vector<double> expected_y{0, 0, -1, -1, 0, 0, 1, 2, 3};
  std::vector<double> expected_theta{
      0,    0,          -0.5 * M_PI, 0,          0.5 * M_PI,
     -M_PI, 0.5 * M_PI, 0.5 * M_PI,  0.5 * M_PI,
  };

  for (int i = 0; i < static_cast<int>(walker.size()); ++i) {
    EXPECT_NEAR(walker.Drx().at(i), expected_x.at(i), 1e-15);
    EXPECT_NEAR(walker.Dry().at(i), expected_y.at(i), 1e-15);
    EXPECT_NEAR(walker.Drtheta().at(i), expected_theta.at(i), 1e-15);
  }
};

TEST_F(MockWalkerTest, testDeadReckoning2) {
  std::vector<double> expected_x{-1, 0, 1, 2, 3};
  std::vector<double> expected_y{2, 2, 2, 2, 2};
  std::vector<double> expected_theta{0, 0, 0, 0, 0};

  for (int i = 0; i < static_cast<int>(walker2.size()); ++i) {
    EXPECT_NEAR(walker2.Drx().at(i), expected_x.at(i), 1e-15);
    EXPECT_NEAR(walker2.Dry().at(i), expected_y.at(i), 1e-15);
    EXPECT_NEAR(walker2.Drtheta().at(i), expected_theta.at(i), 1e-15);
  }
};

TEST_F(MockWalkerTest, testOdometry) {
  std::vector<double> expected_pos{1, 1, 1, 1, 1, 1, 1, 1};
  std::vector<double> expected_theta{
      0, -0.5 * M_PI, 0.5 * M_PI, 0.5 * M_PI, 0.5 * M_PI, -0.5 * M_PI, 0, 0};

  for (int i = 0; i < static_cast<int>(walker.size()) - 1; ++i) {
    EXPECT_NEAR(walker.odom_pos().at(i), expected_pos.at(i), 1e-15);
    EXPECT_NEAR(walker.odom_ang().at(i), expected_theta.at(i), 1e-15);
  }
};

TEST_F(MockWalkerTest, testOdometry2) {
  std::vector<double> expected_pos{1, 1, 1, 1, 1};
  std::vector<double> expected_theta{0, 0, 0, 0, 0};

  for (int i = 0; i < static_cast<int>(walker2.size()) - 1; ++i) {
    EXPECT_NEAR(walker2.odom_pos().at(i), expected_pos.at(i), 1e-15);
    EXPECT_NEAR(walker2.odom_ang().at(i), expected_theta.at(i), 1e-15);
  }
};

void sort_LC_ascending(std::vector<Loopclosure> &vector) {
  std::stable_sort(
      vector.begin(), vector.end(),
      [](Loopclosure lc1, Loopclosure lc2) { return lc1.to < lc2.to; });
  std::stable_sort(
      vector.begin(), vector.end(),
      [](Loopclosure lc1, Loopclosure lc2) { return lc1.from < lc2.from; });
}

TEST_F(MockWalkerTest, testSingleAgentLoopClosure) {
  single_agent_loopclose(walker, params);

  auto LC = walker.LC();

  sort_LC_ascending(LC);

  std::vector<Loopclosure> expected_LC{
      Loopclosure(0, 2, 1.0, -1.0, -0.5 * M_PI, 10000, 10000, 1000000),
      Loopclosure(0, 6, 1.0, 1.0, 0.5 * M_PI, 10000, 10000, 1000000),
      Loopclosure(1, 3, 1.0, -1.0, 0.0, 10000, 10000, 1000000),
      Loopclosure(1, 5, 0.0, 0.0, -M_PI, 10000, 10000, 1000000),
      Loopclosure(2, 4, -1.0, 1.0, -M_PI, 10000, 10000, 1000000),
      Loopclosure(3, 5, -1.0, 1.0, -M_PI, 10000, 10000, 1000000),
      Loopclosure(4, 6, 1.0, 1.0, 0.0, 10000, 10000, 1000000)};

  sort_LC_ascending(expected_LC);

  EXPECT_EQ(LC.size(), expected_LC.size());

  for (int i = 0; i < static_cast<int>(LC.size()); ++i) {
    EXPECT_EQ(LC.at(i).from, expected_LC.at(i).from);
    EXPECT_EQ(LC.at(i).to, expected_LC.at(i).to);
    EXPECT_NEAR(LC.at(i).dx, expected_LC.at(i).dx, 1e-15);
    EXPECT_NEAR(LC.at(i).dy, expected_LC.at(i).dy, 1e-15);
    EXPECT_NEAR(LC.at(i).dtheta, expected_LC.at(i).dtheta, 1e-15);
    EXPECT_NEAR(LC.at(i).I11, expected_LC.at(i).I11, 1e-15);
    EXPECT_NEAR(LC.at(i).I22, expected_LC.at(i).I22, 1e-15);
    EXPECT_NEAR(LC.at(i).I33, expected_LC.at(i).I33, 1e-15);
  }

}

std::vector<std::string> get_text_file_as_string_vector(const std::string& filename){
  std::ifstream file(filename);
  std::vector<std::string> result{};
  if (file.is_open())
  {
    std::string line;
    while(getline(file, line)) {
      result.emplace_back(line);
    }
    file.close();
  }
  return result;
}

void assert_circles_equal(const DiscreteCircle &circle,
                          std::vector<Point> expected) {
  auto coordinates = circle.coordinates();
  ASSERT_TRUE(std::is_permutation(coordinates.begin(), coordinates.end(),
                                  expected.begin()));
}

void sort_multi_LC_ascending(std::vector<MultiLoopclosure> &vector) {
  std::stable_sort(vector.begin(), vector.end(),
                   [](MultiLoopclosure lc1, MultiLoopclosure lc2) {
                     return lc1.to < lc2.to;
                   });
  std::stable_sort(vector.begin(), vector.end(),
                   [](MultiLoopclosure lc1, MultiLoopclosure lc2) {
                     return lc1.from < lc2.from;
                   });
  std::stable_sort(vector.begin(), vector.end(),
                   [](MultiLoopclosure lc1, MultiLoopclosure lc2) {
                     return lc1.ID2 < lc2.ID2;
                   });
  std::stable_sort(vector.begin(), vector.end(),
                   [](MultiLoopclosure lc1, MultiLoopclosure lc2) {
                     return lc1.ID1 < lc2.ID1;
                   });
}

TEST_F(MockWalkerTest, testInterAgentLoopclosure) {
  std::vector<MultiLoopclosure> LCs = inter_agent_loopclose(
      walker, walker2, inter_params, [](Point p, InterLCParams param) {
        (void)p;
        return 1.0 / param.prob_scale;
      });

  write_inter_agent_LC_to_dat(LCs, "inter_agent_lc");

  sort_multi_LC_ascending(LCs);

  std::vector<MultiLoopclosure> expected_LCs{
      MultiLoopclosure(2, 1, 0, 7, 2.0, 0.0, 0.5 * M_PI, 10000, 10000, 1000000),
      MultiLoopclosure(2, 1, 1, 0, 0.0, -2.0, 0.0 * M_PI, 10000, 10000,
                       1000000),
      MultiLoopclosure(2, 1, 1, 6, 1.0, -1.0, 0.5 * M_PI, 10000, 10000,
                       1000000),
      MultiLoopclosure(2, 1, 1, 7, 1.0, 0.0, 0.5 * M_PI, 10000, 10000, 1000000),
      MultiLoopclosure(2, 1, 1, 8, 1.0, 1.0, 0.5 * M_PI, 10000, 10000, 1000000),
      MultiLoopclosure(2, 1, 2, 5, 0.0, -2.0, -1.0 * M_PI, 10000, 10000,
                       1000000),
      MultiLoopclosure(2, 1, 2, 6, 0.0, -1.0, 0.5 * M_PI, 10000, 10000,
                       1000000),
      MultiLoopclosure(2, 1, 2, 7, 0.0, 0.0, 0.5 * M_PI, 10000, 10000, 1000000),
      MultiLoopclosure(2, 1, 2, 8, 0.0, 1.0, 0.5 * M_PI, 10000, 10000, 1000000),
      MultiLoopclosure(2, 1, 3, 4, 0.0, -2.0, 0.5 * M_PI, 10000, 10000,
                       1000000),
      MultiLoopclosure(2, 1, 3, 6, -1.0, -1.0, 0.5 * M_PI, 10000, 10000,
                       1000000),
      MultiLoopclosure(2, 1, 3, 7, -1.0, 0.0, 0.5 * M_PI, 10000, 10000,
                       1000000),
      MultiLoopclosure(2, 1, 3, 8, -1.0, 1.0, 0.5 * M_PI, 10000, 10000,
                       1000000),
      MultiLoopclosure(2, 1, 4, 7, -2.0, 0.0, 0.5 * M_PI, 10000, 10000,
                       1000000)};

  sort_multi_LC_ascending(expected_LCs);

  EXPECT_EQ(LCs.size(), expected_LCs.size());

  for (int i = 0; i < static_cast<int>(LCs.size()); ++i) {
    EXPECT_EQ(LCs.at(i).ID1, expected_LCs.at(i).ID1);
    EXPECT_EQ(LCs.at(i).ID2, expected_LCs.at(i).ID2);
    EXPECT_EQ(LCs.at(i).from, expected_LCs.at(i).from);
    EXPECT_EQ(LCs.at(i).to, expected_LCs.at(i).to);
    EXPECT_NEAR(LCs.at(i).dx, expected_LCs.at(i).dx, 1e-15);
    EXPECT_NEAR(LCs.at(i).dy, expected_LCs.at(i).dy, 1e-15);
    EXPECT_NEAR(LCs.at(i).dtheta, expected_LCs.at(i).dtheta, 1e-15);
    EXPECT_NEAR(LCs.at(i).I11, expected_LCs.at(i).I11, 1e-15);
    EXPECT_NEAR(LCs.at(i).I22, expected_LCs.at(i).I22, 1e-15);
    EXPECT_NEAR(LCs.at(i).I33, expected_LCs.at(i).I33, 1e-15);
  }
}

TEST(TestWalker, testDiscreteCircle) {
  assert_circles_equal(DiscreteCircle(0), std::vector<Point>({Point(0, 0)}));

  assert_circles_equal(
      DiscreteCircle(1),
      std::vector<Point>(
          {Point(0, 0), Point(1, 0), Point(0, 1), Point(-1, 0), Point(0, -1)}));

  assert_circles_equal(
      DiscreteCircle(2),
      std::vector<Point>({Point(0, 0), Point(1, 0), Point(0, 1), Point(-1, 0),
                          Point(0, -1), Point(-2, 0), Point(0, -2), Point(2, 0),
                          Point(0, 2), Point(-1, 1), Point(1, -1), Point(1, 1),
                          Point(-1, -1)}));

  assert_circles_equal(
      DiscreteCircle(3),
      std::vector<Point>(
          {Point(-2, -2), Point(-1, -2), Point(0, -2),  Point(1, -2),
           Point(2, -2),  Point(-2, -1), Point(-1, -1), Point(0, -1),
           Point(1, -1),  Point(2, -1),  Point(-2, 0),  Point(-1, 0),
           Point(0, 0),   Point(1, 0),   Point(2, 0),   Point(-2, 1),
           Point(-1, 1),  Point(0, 1),   Point(1, 1),   Point(2, 1),
           Point(-2, 2),  Point(-1, 2),  Point(0, 2),   Point(1, 2),
           Point(2, 2),   Point(3, 0),   Point(0, 3),   Point(-3, 0),
           Point(0, -3)}));
}

TEST(TestWalker, testPoint) {
  EXPECT_EQ(Point(1, 2), Point(1, 2));
  EXPECT_EQ(Point(0, 0), Point(0, 0));

  EXPECT_NE(Point(0, -2), Point(0, 2));
}

TEST_F(MockWalkerTest, testCenterOfMass) {
  // o-o-o
  walker3.step_forward();
  walker3.step_forward();
  {
    auto [x, y] = center_of_mass(walker3);
    EXPECT_DOUBLE_EQ(x, 1.0);
    EXPECT_DOUBLE_EQ(y, 0.0);
  }

  //     o
  //     |
  //     o
  //     |
  // o-o-o
  walker3.step_left();
  walker3.step_forward();
  {
    auto [x, y] = center_of_mass(walker3);
    EXPECT_DOUBLE_EQ(x, 1.4);
    EXPECT_DOUBLE_EQ(y, 0.6);
  }

  // o-o-o
  //     |
  //     o
  //     |
  // o-o-o
  walker3.step_left();
  walker3.step_forward();
  {
    auto [x, y] = center_of_mass(walker3);
    EXPECT_DOUBLE_EQ(x, 8.0 / 7);
    EXPECT_DOUBLE_EQ(y, 1.0);
  }

  // o-o-o
  // |   |
  // o   o
  //     |
  // o-o-o
  walker3.step_left();
  {
    auto [x, y] = center_of_mass(walker3);
    EXPECT_DOUBLE_EQ(x, 1.0);
    EXPECT_DOUBLE_EQ(y, 1.0);
  }
}

TEST_F(MockWalkerTest, testTranslate) {
  walker.translate(Point(1, 2));

  {
    std::vector<double> expected_x{1, 2, 2, 3, 3, 2, 2, 2, 2};
    std::vector<double> expected_y{2, 2, 1, 1, 2, 2, 3, 4, 5};
    for (int i = 0; i < static_cast<int>(walker.size()); ++i) {
      EXPECT_NEAR(walker.X().at(i), expected_x.at(i), 1e-15);
      EXPECT_NEAR(walker.Y().at(i), expected_y.at(i), 1e-15);
      EXPECT_NEAR(walker.Drx().at(i), expected_x.at(i), 1e-15);
      EXPECT_NEAR(walker.Dry().at(i), expected_y.at(i), 1e-15);
    }
  }

  walker.translate(Point(-1, -2));

  {
    std::vector<double> expected_x{0, 1, 1, 2, 2, 1, 1, 1, 1};
    std::vector<double> expected_y{0, 0, -1, -1, 0, 0, 1, 2, 3};

    for (int i = 0; i < static_cast<int>(walker.size()); ++i) {
      EXPECT_NEAR(walker.X().at(i), expected_x.at(i), 1e-15);
      EXPECT_NEAR(walker.Y().at(i), expected_y.at(i), 1e-15);
      EXPECT_NEAR(walker.Drx().at(i), expected_x.at(i), 1e-15);
      EXPECT_NEAR(walker.Dry().at(i), expected_y.at(i), 1e-15);
    }
  }

  walker.translate(Point(0, 0));
  {
    std::vector<double> expected_x{0, 1, 1, 2, 2, 1, 1, 1, 1};
    std::vector<double> expected_y{0, 0, -1, -1, 0, 0, 1, 2, 3};

    for (int i = 0; i < static_cast<int>(walker.size()); ++i) {
      EXPECT_NEAR(walker.X().at(i), expected_x.at(i), 1e-15);
      EXPECT_NEAR(walker.Y().at(i), expected_y.at(i), 1e-15);
      EXPECT_NEAR(walker.Drx().at(i), expected_x.at(i), 1e-15);
      EXPECT_NEAR(walker.Dry().at(i), expected_y.at(i), 1e-15);
    }
  }
}

TEST_F(MockWalkerTest, testSeed){
  EXPECT_EQ(walker.seed(), 0);
  EXPECT_EQ(walker2.seed(), 1);
  EXPECT_EQ(walker3.seed(), 2);
}

TEST_F(MockWalkerTest, testScaleDoubleAngle){
  EXPECT_EQ(walker_4.scale_double_angle( 0.5*M_PI), 1);
  EXPECT_EQ(walker_4.scale_double_angle(-1.0*M_PI), 2);
  EXPECT_EQ(walker_4.scale_double_angle(-0.5*M_PI), 3);
}

TEST_F(MockWalkerTest, testScaleDoubleLength){
  EXPECT_EQ(walker_4.scale_double_length(0.1), 1);
  EXPECT_EQ(walker_4.scale_double_length(-0.1),-1);
  EXPECT_EQ(walker_4.scale_double_length(0.2), 2);
  EXPECT_EQ(walker_4.scale_double_length(-0.2),-2);
}

TEST_F(MockWalkerTest, testThetaAsDouble){
  EXPECT_EQ(walker_4.theta_as_double(), 0.5*M_PI);
}

TEST_F(MockWalkerTest, testXAsDouble){
  EXPECT_EQ(walker_4.x_as_double(), 0.2);
}

TEST_F(MockWalkerTest, testYAsDouble){
  EXPECT_EQ(walker_4.y_as_double(),-0.2);
}

TEST_F(MockWalkerTest, testWriteG2O){
  EXPECT_EQ(walker.writeg2o("walker1.g2o"), EXIT_SUCCESS);
  EXPECT_EQ(walker2.writeg2o("walker2.g2o"), EXIT_SUCCESS);
  EXPECT_EQ(walker.writeg2o("walker1_3D.g2o", true), EXIT_SUCCESS);
  EXPECT_EQ(walker2.writeg2o("walker2_3D.g2o", true), EXIT_SUCCESS);
}

TEST_F(MockWalkerTest, regressionTestWriteTUM){
  walker.write_ground_truth_as_TUM("walker1.tum");
  auto walker1TUM = get_text_file_as_string_vector("walker1.tum");
  auto walker1_correct = get_text_file_as_string_vector("test_data/walker1.tum");
  for (int i = 0; i < static_cast<int>(walker1TUM.size()); ++i) {
    EXPECT_STREQ(walker1TUM.at(i).c_str(), walker1_correct.at(i).c_str());
  }

  walker2.write_ground_truth_as_TUM("walker2.tum");
  auto walker2TUM = get_text_file_as_string_vector("walker2.tum");
  auto walker2_correct = get_text_file_as_string_vector("test_data/walker2.tum");
  for (int i = 0; i < static_cast<int>(walker2TUM.size()); ++i) {
    EXPECT_STREQ(walker2TUM.at(i).c_str(), walker2_correct.at(i).c_str());
  }
}

TEST_F(MockWalkerTest, regressionTestWriteG2O) {
  single_agent_loopclose(walker, params);

  walker.writeg2o("walker1.g2o");
  auto walker1g2o = get_text_file_as_string_vector("walker1.g2o");
  auto walker1_correct = get_text_file_as_string_vector("test_data/walker1.g2o");
  for (int i = 0; i < static_cast<int>(walker1g2o.size()); ++i) {
    EXPECT_STREQ(walker1g2o.at(i).c_str(), walker1_correct.at(i).c_str());
  }

  walker2.writeg2o("walker2.g2o");
  auto walker2g2o = get_text_file_as_string_vector("walker2.g2o");
  auto walker2_correct = get_text_file_as_string_vector("test_data/walker2.g2o");
  for (int i = 0; i < static_cast<int>(walker2g2o.size()); ++i) {
    EXPECT_STREQ(walker2g2o.at(i).c_str(), walker2_correct.at(i).c_str());
  }

  walker.writeg2o("walker1_3D.g2o", true);
  auto walker1g2o_3D = get_text_file_as_string_vector("walker1_3D.g2o");
  auto walker1_correct_3D = get_text_file_as_string_vector("test_data/walker1_3D.g2o");
  for (int i = 0; i < static_cast<int>(walker1g2o_3D.size()); ++i) {
    EXPECT_STREQ(walker1g2o_3D.at(i).c_str(), walker1_correct_3D.at(i).c_str());
  }

  walker2.writeg2o("walker2_3D.g2o", true);
  auto walker2g2o_3D = get_text_file_as_string_vector("walker2_3D.g2o");
  auto walker2_correct_3D = get_text_file_as_string_vector("test_data/walker2_3D.g2o");
  for (int i = 0; i < static_cast<int>(walker2g2o_3D.size()); ++i) {
    EXPECT_STREQ(walker2g2o_3D.at(i).c_str(), walker2_correct_3D.at(i).c_str());
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
