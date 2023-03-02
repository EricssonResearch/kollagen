#include <filesystem>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

#include "kollagen.h"

template <typename T> using BetweenFactor = gtsam::BetweenFactor<T>;

using Key = gtsam::Key;
using Symbol = gtsam::Symbol;
using Pose2 = gtsam::Pose2;
using Pose2Map = std::map<int, std::vector<std::pair<Key, Pose2>>>;
using FactorMap = std::map<int, std::vector<BetweenFactor<Pose2>>>;

const std::string alphabet =
    "ABCDEFGHIJKLMNOPQRSTUVXYZabcdefghijklmnopqrstuvxyz";

using ID_to_char_map = std::map<int, unsigned char>;

using Data =
    std::tuple<Pose2Map, Pose2Map, FactorMap, FactorMap, ID_to_char_map>;

inline std::array<std::vector<double>, 3> Values_to_XYTheta(const gtsam::Values& values){
  std::array<std::vector<double>, 3> XYTheta{};
  for (const auto& value : values) {
    auto pose2 = value.value.cast<Pose2>();
    XYTheta.at(0).emplace_back(pose2.x());
    XYTheta.at(1).emplace_back(pose2.y());
    XYTheta.at(2).emplace_back(pose2.theta());
  }
  return XYTheta;
}

inline void writeTUM(const gtsam::Values &estimate,
                     const std::string &filename) {
  std::fstream stream(filename.c_str(), std::fstream::out);
  double timestamp{0.0};

  for (const auto &key_value : estimate) {
    const auto &pose = key_value.value.cast<Pose2>();
    stream << std::setprecision(17) << timestamp << " " << pose.x() << " "
           << pose.y() << " " << 0.0 << " " << 0.0 << " " << 0.0 << " "
           << std::sin(pose.theta() / 2) << " " << std::cos(pose.theta() / 2)
           << '\n';
    timestamp += 1;
  }
}

gtsam::Rot3 QuaternionFromTheta(double theta) {
  return gtsam::Rot3::Quaternion(std::cos(theta / 2), 0.0, 0.0,
                                 std::sin(theta / 2));
}

Data get_data(const kollagen::DataGenerator &data_generator) {
  ID_to_char_map ID_to_char{};
  for (int i = 0; const auto &agent : data_generator.Walkers()) {
    ID_to_char[agent.ID()] = alphabet[i];
    i += 1;
  }

  Pose2Map gt_map{};
  Pose2Map pose_map{};
  FactorMap odometry_map{};
  FactorMap loop_closure_map{};

  odometry_map.insert({{0, {}}});
  ;
  loop_closure_map.insert({{0, {}}});
  ;

  for (const auto &agent : data_generator.Walkers()) {
    // Add agent ground truth
    for (int i = 0; i < static_cast<int>(agent.size()); ++i) {
      double x = agent.X(i);
      double y = agent.Y(i);
      double theta = agent.Theta(i);
      gt_map[i].emplace_back(std::make_pair(
          Symbol(ID_to_char.at(agent.ID()), i), Pose2(x, y, theta)));
    }
    // Add agent dead-reckoning
    for (int i = 0; i < static_cast<int>(agent.size()); ++i) {
      double x = agent.Drx(i);
      double y = agent.Dry(i);
      double theta = agent.Drtheta(i);
      pose_map[i].emplace_back(std::make_pair(
          Symbol(ID_to_char.at(agent.ID()), i), Pose2(x, y, theta)));
    }
    // Add agent odometry
    for (int i = 1; i < static_cast<int>(agent.size()); ++i) {
      odometry_map.insert({{i, {}}});
      loop_closure_map.insert({{i, {}}});
      double theta = agent.odom_ang(i - 1);
      double x = agent.odom_pos(i - 1) * std::cos(theta);
      double y = agent.odom_pos(i - 1) * std::sin(theta);
      gtsam::SharedNoiseModel model = gtsam::noiseModel::Diagonal::Variances(
          gtsam::Vector3(1 / agent.Params().I11est, 1 / agent.Params().I22est,
                         1 / agent.Params().I33est));
      odometry_map[i].emplace_back(BetweenFactor<Pose2>(
          Symbol(ID_to_char.at(agent.ID()), i - 1),
          Symbol(ID_to_char.at(agent.ID()), i), Pose2(x, y, theta), model));
    }
    // Add agent loop closures
    for (const auto &lc : agent.LC()) {
      gtsam::SharedNoiseModel model = gtsam::noiseModel::Diagonal::Variances(
          gtsam::Vector3(1 / lc.I11, 1 / lc.I22, 1 / lc.I33));
      int i = std::max(lc.from, lc.to);
      loop_closure_map[i].emplace_back(
          BetweenFactor<Pose2>(Symbol(ID_to_char.at(agent.ID()), lc.from),
                               Symbol(ID_to_char.at(agent.ID()), lc.to),
                               Pose2(lc.dx, lc.dy, lc.dtheta), model));
    }
  }
  // Add inter-agent loop-closures
  for (const auto &lc : data_generator.LCs()) {
    gtsam::SharedNoiseModel model = gtsam::noiseModel::Diagonal::Variances(
        gtsam::Vector3(1 / lc.I11, 1 / lc.I22, 1 / lc.I33));
    int i = std::max(lc.from, lc.to);
    loop_closure_map[i].emplace_back(
        BetweenFactor<Pose2>(Symbol(ID_to_char.at(lc.ID1), lc.from),
                             Symbol(ID_to_char.at(lc.ID2), lc.to),
                             Pose2(lc.dx, lc.dy, lc.dtheta), model));
  }
  return {gt_map, pose_map, odometry_map, loop_closure_map, ID_to_char};
}

int main() {

  std::string filename = "data/iSAM2example.json";
  auto data_generator =
    kollagen::DataGenerator(kollagen::get_params_from_json(filename));
  data_generator.generate();

  auto [gt_map, pose_map, odometry_map, loop_closure_map, ID_to_char] =
      get_data(data_generator);

  gtsam::NonlinearFactorGraph g2o_graph;

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;

  graph.addPrior(pose_map.at(0).at(0).first, pose_map.at(0).at(0).second);

  gtsam::ISAM2 isam{};

  int number_of_agents = static_cast<int>(data_generator.Walkers().size());
  int number_of_steps = static_cast<int>(pose_map.size());

  std::vector<std::vector<double>> APE(number_of_agents);
  for (int i = 0; i < number_of_agents; ++i) {
    APE.at(i).reserve(number_of_steps);
  }
  std::vector<int> timesteps{};

  for (const auto &indexed_pose : pose_map.at(0)) {
    auto index = indexed_pose.first;
    auto pose = indexed_pose.second;
    initial.insert(index, pose);
  }

  bool all_connected = false;
  gtsam::Values latest_result;
  latest_result = initial;
  gtsam::Values online_result;
  for (int i = 1; i < number_of_steps; ++i) {
    for (const auto &odometry : odometry_map.at(i)) {
      auto key1 = odometry.key1();
      auto key2 = odometry.key2();
      graph.add(odometry);
      g2o_graph.add(odometry);

      auto latest_pose = latest_result.at<Pose2>(key1);
      auto pose = latest_pose.compose(odometry.measured());
      initial.insert(key2, pose);
    }

    for (const auto &loop_closure : loop_closure_map.at(i)) {
      graph.add(loop_closure);
      g2o_graph.add(loop_closure);
    }

    if (all_connected) {
      isam.update(graph, initial);
      latest_result = isam.calculateEstimate();

      for (int j = 0; j < number_of_agents; ++j) {
        auto KeyPosePair = gt_map.at(i).at(j);
        auto estimate = isam.calculateEstimate<Pose2>(KeyPosePair.first);
        online_result.insert(KeyPosePair.first, estimate);
        auto ground_truth = KeyPosePair.second;
        APE.at(j).emplace_back(std::hypot(estimate.x() - ground_truth.x(),
                                          estimate.y() - ground_truth.y()));
      }
      timesteps.emplace_back(i);

      graph.resize(0);
      initial.clear();
    } else {
      latest_result = initial;
      try {
        if (i == 0 || i < 3260) {
          continue;
        }
        gtsam::GaussNewtonOptimizer optimizer{graph, initial};
        gtsam::Values result = optimizer.optimize();
        if (!all_connected) {
          std::cout << "All connected at timestep " << i << std::endl;
        }
        all_connected = true;
        for (const auto& value : result) {
          online_result.insert(value.key, value.value.cast<Pose2>());
        }
      } catch (gtsam::IndeterminantLinearSystemException const &) {
      }
    }
  }
  std::cout << "Done, now saving..." << std::endl;
  gtsam::Values offline_result = isam.calculateEstimate();

  auto all_agents_TUM = kollagen::open_file("iSAM2_all.tum");;
  int offset = 0;

  for (int i = 0; auto const &agent : data_generator.Walkers()) {
    unsigned char symbol = ID_to_char.at(agent.ID());
    gtsam::Values temp_result;
    uint k = 0;
    while (true) {
      try {
        temp_result.insert(k, online_result.at(Symbol(symbol, k)));
        k += 1;
      } catch (gtsam::ValuesKeyDoesNotExist const &) {
        std::string output_file =
            "agent" + std::to_string(agent.ID()) + "_" + "test" + ".tum";
        std::cout << "Saving to " << output_file << '\n';
        writeTUM(temp_result, output_file);
        std::string GT = "agent" + std::to_string(agent.ID()) + "_" + "test" +
                         "_GT" + ".tum";
        agent.write_ground_truth_as_TUM(GT);
        kollagen::write_TUM_vertices(all_agents_TUM, Values_to_XYTheta(temp_result), offset);
        offset += static_cast<int>(temp_result.size());

        break;
      }
    }
    kollagen::save_binary("APE_agent" + std::to_string(agent.ID()) + ".bin", APE.at(i));
    i++;
  }

  kollagen::save_binary("APE_time.bin", timesteps);

  return EXIT_SUCCESS;
}
