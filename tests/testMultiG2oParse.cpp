#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "kollagen.h"

class TestMultiG2OGetAgentEntries2D: public ::testing::Test {
protected:
  void SetUp() override {
    bool _3D = false;
    entries = kollagen::multig2o::helper::get_agent_entries("test_data/multig2o_test/agent1/posegraph.g2o","test_data/multig2o_test/agent1/agent1_GT.tum", _3D);

  }
  kollagen::multig2o::AgentEntries entries;
};

class TestMultiG2OGetInterAgentLCs2D: public ::testing::Test {
protected:
  void SetUp() override {
    bool _3D = false;
    entries = kollagen::multig2o::helper::get_inter_agent_lcs("test_data/multig2o_test/inter_agent_lc.dat", _3D);

  }
  std::vector<kollagen::multig2o::InterAgentLC> entries;
};

class TestMultiG2OGetAgentEntries3D: public ::testing::Test {
protected:
  void SetUp() override {
    bool _3D = true;
    entries = kollagen::multig2o::helper::get_agent_entries("test_data/multig2o_test_3D/agent1/posegraph.g2o","test_data/multig2o_test_3D/agent1/agent1_GT.tum", _3D);

  }
  kollagen::multig2o::AgentEntries entries;
};

class TestMultiG2OGetInterAgentLCs3D: public ::testing::Test {
protected:
  void SetUp() override {
    bool _3D = true;
    entries = kollagen::multig2o::helper::get_inter_agent_lcs("test_data/multig2o_test_3D/inter_agent_lc.dat", _3D);

  }
  std::vector<kollagen::multig2o::InterAgentLC> entries;
};

TEST(MultiG2OParse, testIsAgentDirectory){
  std::vector<std::string> true_dirs = {
    "test_data/multig2o_test/agent1",
    "test_data/multig2o_test/agent2"
  };
  for (const auto& true_dir : true_dirs) {
    ASSERT_TRUE(kollagen::multig2o::helper::is_agent_directory(std::filesystem::directory_entry(true_dir)));
  }
}
TEST(MultiG2OParse, testIsAgentDirectory2){
  std::vector<std::string> false_dirs = {
    "test_data/multig2o_test/agent3",
    "test_data/multig2o_test/agent4",
    "test_data/multig2o_test",
    "t",
    ""
  };
  for (const auto& false_dir : false_dirs) {
    ASSERT_FALSE(kollagen::multig2o::helper::is_agent_directory(std::filesystem::directory_entry(false_dir)));
  }
}
TEST(MultiG2OParse, testGetSortedAgentDirs){
  std::vector<std::string> correct = {
    "test_data/multig2o_test/agent1",
    "test_data/multig2o_test/agent2"
  };
  auto sorted_dirs = kollagen::multig2o::helper::get_sorted_agent_dirs("test_data/multig2o_test");
  for (int i = 0; const auto& dir : sorted_dirs) {
    ASSERT_EQ(dir.string(), correct.at(i));
    i += 1;
  }
}

TEST(MultiG2OParse, testExtractG2oPaths){
  kollagen::multig2o::MultiG2OPaths paths = kollagen::multig2o::helper::extract_paths("test_data/multig2o_test");
  std::vector<std::string> correct_g2o = {
    "test_data/multig2o_test/agent1/posegraph.g2o",
    "test_data/multig2o_test/agent2/posegraph.g2o"
  };
  std::vector<std::string> correct_gt = {
    "test_data/multig2o_test/agent1/agent1_GT.tum",
    "test_data/multig2o_test/agent2/agent2_GT.tum"
  };
  std::string correct_lc = "test_data/multig2o_test/inter_agent_lc.dat";

  EXPECT_EQ(correct_g2o.size(), paths.g2o_files.size());
  EXPECT_EQ(correct_gt.size(), paths.ground_truth_files.size());

  EXPECT_EQ(paths.inter_agent_lc.string(), correct_lc);

  for (int i = 0; const auto& correct : correct_g2o) {
    EXPECT_EQ(paths.g2o_files.at(i).string(), correct);
    i += 1;
  }

  for (int i = 0; const auto& correct : correct_gt) {
    EXPECT_EQ(paths.ground_truth_files.at(i).string(), correct);
    i += 1;
  }
}

TEST(MultiG2OParse, testParseTUM){
  auto ground_truth = kollagen::multig2o::helper::parse_TUM("test_data/multig2o_test/agent1/agent1_GT.tum");
  std::vector<std::array<double, 8>> correct_gt = {
    {0, 0, 0, 0.0, 0.0, 0.0, 0, 1},
    {1, 1, 0, 0.0, 0.0, 0.0, 0, 1},
    {2, 1,-1, 0.0, 0.0, 0.0, -0.70710678118654746, 0.70710678118654757},
    {3, 2,-1, 0.0, 0.0, 0.0, 0, 1}
  };
  EXPECT_EQ(ground_truth.size(), correct_gt.size());

  for (int i = 0; const auto& gt_row: correct_gt) {
    EXPECT_EQ(gt_row, ground_truth.at(i));
    i += 1;
  }
}

TEST_F(TestMultiG2OGetAgentEntries2D, testGetAgentEntries2DOdometry){
  std::vector<kollagen::multig2o::Edge> correct_odometry = {
    {0, 1, {1, 0, 0}, {1, 0.000000, 0.000000, 1, 0.000000, 1}},
    {1, 2, {6.123233995736766e-17, -1, -1.5707963267948966}, {1, 0.000000, 0.000000, 1, 0.000000, 1}},
    {2, 3, {6.123233995736766e-17, 1, 1.5707963267948966},   {1, 0.000000, 0.000000, 1, 0.000000, 1}}
  };

  EXPECT_EQ(entries.odometry.size(), correct_odometry.size());

  for (int i = 0; const auto& odometry_row: correct_odometry) {
    auto test = entries.odometry.at(i);
    EXPECT_EQ(odometry_row.from, test.from);
    EXPECT_EQ(odometry_row.to, test.to);
    EXPECT_EQ(odometry_row.transform, test.transform);
    EXPECT_EQ(odometry_row.information_matrix, test.information_matrix);
    i += 1;
  }
}

TEST_F(TestMultiG2OGetAgentEntries2D, testGetAgentEntries2DLC){
  std::vector<kollagen::multig2o::Edge> correct_lc = {
    {1, 3, {1, -1, 0}, {10000, 0.000000, 0.000000, 10000, 0.000000, 1000000}},
  };

  EXPECT_EQ(entries.loop_closures.size(), correct_lc.size());

  for (int i = 0; const auto& lc: correct_lc) {
    auto test = entries.loop_closures.at(i);
    EXPECT_EQ(lc.from, test.from);
    EXPECT_EQ(lc.to, test.to);
    EXPECT_EQ(lc.transform, test.transform);
    EXPECT_EQ(lc.information_matrix, test.information_matrix);
    i += 1;
  }
}

TEST_F(TestMultiG2OGetAgentEntries2D, testGetAgentEntries2DVertices){
  std::vector<kollagen::multig2o::Vertex> correct_vertices = {
    {0, {0, 0, 0                  }},
    {1, {1, 0, 0                  }},
    {2, {1,-1, -1.5707963267948966}},
    {3, {2,-1, 0                  }},
  };

  EXPECT_EQ(entries.vertices.size(), correct_vertices.size());

  for (int i = 0; const auto& vertex_row : correct_vertices) {
    auto test = entries.vertices.at(i);
    EXPECT_EQ(vertex_row.ID, test.ID);
    EXPECT_EQ(vertex_row.pose.at(0), test.pose.at(0));
    EXPECT_EQ(vertex_row.pose.at(1), test.pose.at(1));
    EXPECT_EQ(vertex_row.pose.at(2), test.pose.at(2));
    i += 1;
  }
}

TEST_F(TestMultiG2OGetInterAgentLCs2D, testGetInterAgentLCs2D){
  std::vector<kollagen::multig2o::InterAgentLC> correct_lc = {
    {2, 0, 1, 0, {1, -2, 0}, {1, 0.000000, 0.000000, 1, 0.000000, 1}},
  };

  EXPECT_EQ(entries.size(), correct_lc.size());

  for (int i = 0; const auto& lc : correct_lc) {
    auto test = entries.at(i);
    EXPECT_EQ(lc.from_agent, test.from_agent);
    EXPECT_EQ(lc.from_index, test.from_index);
    EXPECT_EQ(lc.to_agent, test.to_agent);
    EXPECT_EQ(lc.to_index, test.to_index);
    EXPECT_EQ(lc.edge.from, test.edge.from);
    EXPECT_EQ(lc.edge.to, test.edge.to);
    EXPECT_EQ(lc.edge.transform, test.edge.transform);
    EXPECT_EQ(lc.edge.information_matrix, test.edge.information_matrix);

    EXPECT_EQ(lc.edge.from, test.from_index);
    EXPECT_EQ(lc.edge.to, test.to_index);
    i += 1;
  }
}

TEST_F(TestMultiG2OGetAgentEntries3D, testGetAgentEntries3DOdometry){
  std::vector<kollagen::multig2o::Edge> correct_odometry = {
    {0  ,  1,  {  0.28660034220463471  ,    -0.00026702747031739167,   0.0 ,  0.0 ,  0.0 ,  -0.0004658532180006681    ,0.99999989149038371    } ,{1890.359168241966   ,0.0   ,0.0   ,0.0   ,0.0   ,0.0  , 1890.359168241966 ,  0.0  , 0.0 ,  0.0 ,  0.0 ,  1000000 ,  0.0 ,  0.0 ,  0.0 ,  189035.91682419661  , 0.0 ,  0.0 ,  189035.91682419661 ,  0.0 ,  189035.91682419661}},
    {1  ,  2,  {  0.28226785642980329  ,    0.0010509744534258326  ,   0.0 ,  0.0 ,  0.0 ,  0.0018616519129953351     ,0.99999826712457596    } ,{1890.359168241966   ,0.0   ,0.0   ,0.0   ,0.0   ,0.0  , 1890.359168241966 ,  0.0  , 0.0 ,  0.0 ,  0.0 ,  1000000 ,  0.0 ,  0.0 ,  0.0 ,  189035.91682419661  , 0.0 ,  0.0 ,  189035.91682419661 ,  0.0 ,  189035.91682419661}},
    {2  ,  3,  {  0.29259937495951516  ,    0.00027003105678605378 ,   0.0 ,  0.0 ,  0.0 ,  0.0004614346332423154     ,0.99999989353903396    } ,{1890.359168241966   ,0.0   ,0.0   ,0.0   ,0.0   ,0.0  , 1890.359168241966 ,  0.0  , 0.0 ,  0.0 ,  0.0 ,  1000000 ,  0.0 ,  0.0 ,  0.0 ,  189035.91682419661  , 0.0 ,  0.0 ,  189035.91682419661 ,  0.0 ,  189035.91682419661}},
    {3  ,  4,  {  0.27236578189264238  ,    0.0012376155300146889  ,   0.0 ,  0.0 ,  0.0 ,  0.0022719556380044331     ,0.99999741910545892    } ,{1890.359168241966   ,0.0   ,0.0   ,0.0   ,0.0   ,0.0  , 1890.359168241966 ,  0.0  , 0.0 ,  0.0 ,  0.0 ,  1000000 ,  0.0 ,  0.0 ,  0.0 ,  189035.91682419661  , 0.0 ,  0.0 ,  189035.91682419661 ,  0.0 ,  189035.91682419661}},
    {4  ,  5,  {  -0.25433167664632361 ,    -0.00071351644112970078,   0.0 ,  0.0 ,  0.0 ,  -0.99999901618209119      ,0.0014027240818501579  } ,{1890.359168241966   ,0.0   ,0.0   ,0.0   ,0.0   ,0.0  , 1890.359168241966 ,  0.0  , 0.0 ,  0.0 ,  0.0 ,  1000000 ,  0.0 ,  0.0 ,  0.0 ,  189035.91682419661  , 0.0 ,  0.0 ,  189035.91682419661 ,  0.0 ,  189035.91682419661}},
    {5  ,  6,  {  0.25216323512353289  ,    -7.2622970441349821e-05,   0.0 ,  0.0 ,  0.0 ,  -0.00014399991368073579   ,0.99999998963201242    } ,{1890.359168241966   ,0.0   ,0.0   ,0.0   ,0.0   ,0.0  , 1890.359168241966 ,  0.0  , 0.0 ,  0.0 ,  0.0 ,  1000000 ,  0.0 ,  0.0 ,  0.0 ,  189035.91682419661  , 0.0 ,  0.0 ,  189035.91682419661 ,  0.0 ,  189035.91682419661}},
    {6  ,  7,  {  0.27154365690039667  ,    0.00089958203879594867 ,   0.0 ,  0.0 ,  0.0 ,  0.0016564156695242907     ,0.99999862814262386    } ,{1890.359168241966   ,0.0   ,0.0   ,0.0   ,0.0   ,0.0  , 1890.359168241966 ,  0.0  , 0.0 ,  0.0 ,  0.0 ,  1000000 ,  0.0 ,  0.0 ,  0.0 ,  189035.91682419661  , 0.0 ,  0.0 ,  189035.91682419661 ,  0.0 ,  189035.91682419661}},
    {7  ,  8,  {  0.23804065830941221  ,    0.0004013683350272312  ,   0.0 ,  0.0 ,  0.0 ,  0.00084306586522401152    ,0.99999964461991031    } ,{1890.359168241966   ,0.0   ,0.0   ,0.0   ,0.0   ,0.0  , 1890.359168241966 ,  0.0  , 0.0 ,  0.0 ,  0.0 ,  1000000 ,  0.0 ,  0.0 ,  0.0 ,  189035.91682419661  , 0.0 ,  0.0 ,  189035.91682419661 ,  0.0 ,  189035.91682419661}},
    {8  ,  9,  {  -0.0013187550215991756 ,  -0.25485598852081692   ,   0.0 ,  0.0 ,  0.0 ,  -0.70893386199126718      ,0.70527496717390092    } ,{1890.359168241966   ,0.0   ,0.0   ,0.0   ,0.0   ,0.0  , 1890.359168241966 ,  0.0  , 0.0 ,  0.0 ,  0.0 ,  1000000 ,  0.0 ,  0.0 ,  0.0 ,  189035.91682419661  , 0.0 ,  0.0 ,  189035.91682419661 ,  0.0 ,  189035.91682419661}},
    {9  ,  10 ,{  0.24856410284041186  ,    0.0002403959858711792  ,   0.0 ,  0.0 ,  0.0 ,  0.00048356922581155787    ,0.99999988308039511    } ,{1890.359168241966   ,0.0   ,0.0   ,0.0   ,0.0   ,0.0  , 1890.359168241966 ,  0.0  , 0.0 ,  0.0 ,  0.0 ,  1000000 ,  0.0 ,  0.0 ,  0.0 ,  189035.91682419661  , 0.0 ,  0.0 ,  189035.91682419661 ,  0.0 ,  189035.91682419661}},
    {10 ,  11 ,{  0.22082665660653311  ,    -7.6365097323481283e-05,   0.0 ,  0.0 ,  0.0 ,  -0.00017290732711432416   ,0.99999998505152798    } ,{1890.359168241966   ,0.0   ,0.0   ,0.0   ,0.0   ,0.0  , 1890.359168241966 ,  0.0  , 0.0 ,  0.0 ,  0.0 ,  1000000 ,  0.0 ,  0.0 ,  0.0 ,  189035.91682419661  , 0.0 ,  0.0 ,  189035.91682419661 ,  0.0 ,  189035.91682419661}},
    {11 ,  12 ,{  0.24237476229946212  ,    0.00012057755413520462 ,   0.0 ,  0.0 ,  0.0 ,  0.00024874195192724739    ,0.99999996906372024    } ,{1890.359168241966   ,0.0   ,0.0   ,0.0   ,0.0   ,0.0  , 1890.359168241966 ,  0.0  , 0.0 ,  0.0 ,  0.0 ,  1000000 ,  0.0 ,  0.0 ,  0.0 ,  189035.91682419661  , 0.0 ,  0.0 ,  189035.91682419661 ,  0.0 ,  189035.91682419661}},
    {12 ,  13 ,{  -0.23611223891249722 ,    -0.00068183292997908349,   0.0 ,  0.0 ,  0.0 ,  -0.99999895761904289      ,0.0014438700868065581  } ,{1890.359168241966   ,0.0   ,0.0   ,0.0   ,0.0   ,0.0  , 1890.359168241966 ,  0.0  , 0.0 ,  0.0 ,  0.0 ,  1000000 ,  0.0 ,  0.0 ,  0.0 ,  189035.91682419661  , 0.0 ,  0.0 ,  189035.91682419661 ,  0.0 ,  189035.91682419661}},
    {13 ,  14 ,{  0.25652845263757401  ,    0.00031927387424937457 ,   0.0 ,  0.0 ,  0.0 ,  0.00062229683590030594    ,0.99999980637330532    } ,{1890.359168241966   ,0.0   ,0.0   ,0.0   ,0.0   ,0.0  , 1890.359168241966 ,  0.0  , 0.0 ,  0.0 ,  0.0 ,  1000000 ,  0.0 ,  0.0 ,  0.0 ,  189035.91682419661  , 0.0 ,  0.0 ,  189035.91682419661 ,  0.0 ,  189035.91682419661}},
    {14 ,  15 ,{  0.23498143378746408  ,    -0.00011673464691620459,   0.0 ,  0.0 ,  0.0 ,  -0.00024839119038297071   ,0.99999996915090783    } ,{1890.359168241966   ,0.0   ,0.0   ,0.0   ,0.0   ,0.0  , 1890.359168241966 ,  0.0  , 0.0 ,  0.0 ,  0.0 ,  1000000 ,  0.0 ,  0.0 ,  0.0 ,  189035.91682419661  , 0.0 ,  0.0 ,  189035.91682419661 ,  0.0 ,  189035.91682419661}},
    {15 ,  16 ,{  0.22862641063747313  ,    -0.00083826807348389462,   0.0 ,  0.0 ,  0.0 ,  -0.0018332611818667613    ,0.9999983195753076     } ,{1890.359168241966   ,0.0   ,0.0   ,0.0   ,0.0   ,0.0  , 1890.359168241966 ,  0.0  , 0.0 ,  0.0 ,  0.0 ,  1000000 ,  0.0 ,  0.0 ,  0.0 ,  189035.91682419661  , 0.0 ,  0.0 ,  189035.91682419661 ,  0.0 ,  189035.91682419661}}
  };

  EXPECT_EQ(entries.odometry.size(), correct_odometry.size());

  for (int i = 0; const auto& odometry_row: correct_odometry) {
    auto test = entries.odometry.at(i);
    EXPECT_EQ(odometry_row.from, test.from);
    EXPECT_EQ(odometry_row.to, test.to);
    EXPECT_EQ(odometry_row.transform, test.transform);
    EXPECT_EQ(odometry_row.information_matrix, test.information_matrix);
    i += 1;
  }
}

TEST_F(TestMultiG2OGetAgentEntries3D, testGetAgentEntries3DVertices){
  std::vector<kollagen::multig2o::Vertex> correct_vertices = {
    {0   , {0                    , 0                       ,  0.0 ,  0.0 ,  0.0 ,  0                      ,  1}},
    {1   , {0.28660034220463471  , -0.00026702747031739167 ,  0.0 ,  0.0 ,  0.0 ,  -0.0004658532180006681 ,  0.99999989149038371}},
    {2   , {0.56886905531889354  , 0.00052095577697020674  ,  0.0 ,  0.0 ,  0.0 ,  0.001395799300253125   ,  0.99999902587168221}},
    {3   , {0.86146653634403048  , 0.0016078049915356646   ,  0.0 ,  0.0 ,  0.0 ,  0.0018572333354007556  ,  0.99999827534068175}},
    {4   , {1.133825842211849    , 0.0038571038579772775   ,  0.0 ,  0.0 ,  0.0 ,  0.0041291802617323507  ,  0.99999147489884443}},
    {5   , {0.87950873076741543  , 0.0010432669754649642   ,  0.0 ,  0.0 ,  0.0 ,  -0.99998469898873121   ,  0.0055318883228562387}},
    {6   , {0.62736012546378772  , -0.0016739495225960048  ,  0.0 ,  0.0 ,  0.0 ,  -0.99998548521234321   ,  0.0053878905551653037}},
    {7   , {0.35584192760149058  , -0.0054995318698116136  ,  0.0 ,  0.0 ,  0.0 ,  -0.99997518878853808   ,  0.0070442747907505342}},
    {8   , {0.11783054786483249  , -0.0092544247805395365  ,  0.0 ,  0.0 ,  0.0 ,  -0.99996889462964444   ,  0.0078873172350939838}},
    {9   , {0.11512900379775405  , 0.24559065690514534     ,  0.0 ,  0.0 ,  0.0 ,  0.71084461560306988    ,  0.70334908293686138}},
    {10  , {0.11225380825594418  , 0.4941382464361751      ,  0.0 ,  0.0 ,  0.0 ,  0.71118465046290924    ,  0.70300525812112502}},
    {11  , {0.10977581902651104  , 0.71495101252778182     ,  0.0 ,  0.0 ,  0.0 ,  0.71106308507165639    ,  0.70312821664926684}},
    {12  , {0.10693545959773573  , 0.95730916134267663     ,  0.0 ,  0.0 ,  0.0 ,  0.71123796055907429    ,  0.70295132367737145}},
    {13  , {0.1103842150679314   , 0.7212211261817999      ,  0.0 ,  0.0 ,  0.0 ,  -0.70192365571844539   ,  0.71225218956690706}},
    {14  , {0.11445040260352612  , 0.46472470307348401     ,  0.0 ,  0.0 ,  0.0 ,  -0.70148028752335745   ,  0.712688856525867}},
    {15  , {0.1180583292542563   , 0.22977094013900129     ,  0.0 ,  0.0 ,  0.0 ,  -0.70165729151677247   ,  0.71251459301641462}},
    {16  , {0.12073051060145405  , 0.0011586093666100805   ,  0.0 ,  0.0 ,  0.0 ,  -0.70296233777942485   ,  0.71122707461378731}}
  };

  EXPECT_EQ(entries.vertices.size(), correct_vertices.size());

  for (int i = 0; const auto& vertex_row : correct_vertices) {
    auto test = entries.vertices.at(i);
    EXPECT_EQ(vertex_row.ID, test.ID);
    EXPECT_EQ(vertex_row.pose.at(0), test.pose.at(0));
    EXPECT_EQ(vertex_row.pose.at(1), test.pose.at(1));
    EXPECT_EQ(vertex_row.pose.at(2), test.pose.at(2));
    i += 1;
  }
}

TEST_F(TestMultiG2OGetInterAgentLCs3D, testGetInterAgentLCs3D){
  std::vector<kollagen::multig2o::InterAgentLC> correct_lc = {
  {2 , 8  , 1 , 16 , {-0.014624 , -0.018364 , 0.0 , 0.0 , 0.0 , -1.000000 , 0.000441 }, {1600.000000 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1600.000000 , 0.0 , 0.0 , 0.0 , 0.0 , 1000000.000000 , 0.0 , 0.0 , 0.0 , 1600.000000 , 0.0 , 0.0 , 1600.000000 , 0.0 , 1600.000000}} ,
  {2 , 9  , 1 , 15 , {0.035083  , 0.008447  , 0.0 , 0.0 , 0.0 , -1.000000 , 0.000722 }, {1600.000000 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1600.000000 , 0.0 , 0.0 , 0.0 , 0.0 , 1000000.000000 , 0.0 , 0.0 , 0.0 , 1600.000000 , 0.0 , 0.0 , 1600.000000 , 0.0 , 1600.000000}} ,
  {2 , 10 , 1 , 14 , {-0.022942 , 0.024511  , 0.0 , 0.0 , 0.0 , -1.000000 , 0.000515 }, {1600.000000 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1600.000000 , 0.0 , 0.0 , 0.0 , 0.0 , 1000000.000000 , 0.0 , 0.0 , 0.0 , 1600.000000 , 0.0 , 0.0 , 1600.000000 , 0.0 , 1600.000000}} ,
  {2 , 11 , 1 , 13 , {-0.031795 , -0.038002 , 0.0 , 0.0 , 0.0 , -0.999999 , 0.001562 }, {1600.000000 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1600.000000 , 0.0 , 0.0 , 0.0 , 0.0 , 1000000.000000 , 0.0 , 0.0 , 0.0 , 1600.000000 , 0.0 , 0.0 , 1600.000000 , 0.0 , 1600.000000}} ,
  {2 , 12 , 1 , 12 , {0.047890  , -0.032968 , 0.0 , 0.0 , 0.0 , -0.001639 , 0.999999 }, {1600.000000 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1600.000000 , 0.0 , 0.0 , 0.0 , 0.0 , 1000000.000000 , 0.0 , 0.0 , 0.0 , 1600.000000 , 0.0 , 0.0 , 1600.000000 , 0.0 , 1600.000000}} ,
  {2 , 14 , 1 , 12 , {-0.514714 , 0.012303  , 0.0 , 0.0 , 0.0 , 0.001019  , 0.999999 }, {1600.000000 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1600.000000 , 0.0 , 0.0 , 0.0 , 0.0 , 1000000.000000 , 0.0 , 0.0 , 0.0 , 1600.000000 , 0.0 , 0.0 , 1600.000000 , 0.0 , 1600.000000}}
  };

  EXPECT_EQ(entries.size(), correct_lc.size());

  for (int i = 0; const auto& lc : correct_lc) {
    auto test = entries.at(i);
    EXPECT_EQ(lc.from_agent, test.from_agent);
    EXPECT_EQ(lc.from_index, test.from_index);
    EXPECT_EQ(lc.to_agent, test.to_agent);
    EXPECT_EQ(lc.to_index, test.to_index);
    EXPECT_EQ(lc.edge.from, test.edge.from);
    EXPECT_EQ(lc.edge.to, test.edge.to);
    EXPECT_EQ(lc.edge.transform, test.edge.transform);
    EXPECT_EQ(lc.edge.information_matrix, test.edge.information_matrix);

    EXPECT_EQ(lc.edge.from, test.from_index);
    EXPECT_EQ(lc.edge.to, test.to_index);
    i += 1;
  }
}

TEST_F(TestMultiG2OGetAgentEntries3D, testGetAgentEntries3DLC){
  std::vector<kollagen::multig2o::Edge> correct_lc = {
    {1    ,7    ,{0.00070534665879937544   ,-0.008468276345526219   ,0.0   ,0.0   ,0.0   ,-0.99999999645008819   ,-8.4260450603654741e-05   },{1890.359168241966   ,0.0   ,0.0   ,0.0   ,0.0   ,0.0   ,1890.359168241966   ,0.0   ,0.0   ,0.0   ,0.0   ,1000000   ,0.0   ,0.0   ,0.0   ,189035.91682419661   ,0.0   ,0.0   ,189035.91682419661   ,0.0   ,189035.91682419661}},
    {0    ,8    ,{0.010073736378136359     ,-0.039903138891562245   ,0.0   ,0.0   ,0.0   ,-0.9999982167639998    ,0.0018885096823948941     },{1890.359168241966   ,0.0   ,0.0   ,0.0   ,0.0   ,0.0   ,1890.359168241966   ,0.0   ,0.0   ,0.0   ,0.0   ,1000000   ,0.0   ,0.0   ,0.0   ,189035.91682419661   ,0.0   ,0.0   ,189035.91682419661   ,0.0   ,189035.91682419661}},
    {11   ,13   ,{-0.013991981638450963    ,-0.02669565735519381    ,0.0   ,0.0   ,0.0   ,-0.99999706167073199   ,-0.0024241802536616912    },{1890.359168241966   ,0.0   ,0.0   ,0.0   ,0.0   ,0.0   ,1890.359168241966   ,0.0   ,0.0   ,0.0   ,0.0   ,1000000   ,0.0   ,0.0   ,0.0   ,189035.91682419661   ,0.0   ,0.0   ,189035.91682419661   ,0.0   ,189035.91682419661}},
    {0    ,16   ,{-0.034727147355296577    ,0.0058708814894198455   ,0.0   ,0.0   ,0.0   ,-0.70800674751665493   ,0.70620566796853701       },{1890.359168241966   ,0.0   ,0.0   ,0.0   ,0.0   ,0.0   ,1890.359168241966   ,0.0   ,0.0   ,0.0   ,0.0   ,1000000   ,0.0   ,0.0   ,0.0   ,189035.91682419661   ,0.0   ,0.0   ,189035.91682419661   ,0.0   ,189035.91682419661}}
  };

  EXPECT_EQ(entries.loop_closures.size(), correct_lc.size());

  for (int i = 0; const auto& lc: correct_lc) {
    auto test = entries.loop_closures.at(i);
    EXPECT_EQ(lc.from, test.from);
    EXPECT_EQ(lc.to, test.to);
    EXPECT_EQ(lc.transform, test.transform);
    EXPECT_EQ(lc.information_matrix, test.information_matrix);
    i += 1;
  }
}
