#include "DataGeneratorParams.h"
#include "Functions.h"
#include "Walker-impl.h"
#include "Walker.h"

#include <filesystem>

#ifndef DATAGENERATOR_H
#define DATAGENERATOR_H

namespace kollagen
{

/**
 * \brief Data Generator.
 * Can generates and save data given a set of parameters.
 */

class DataGenerator {
public:
  DataGenerator() = default;
  explicit DataGenerator(DataGeneratorParams params);

  /*! \brief Set parameters of Data Generator
   *
   * For use with the default constructor. Parameters can only be set once.
   *
   * \param params Data Generator Parameters
   * \return EXIT_FAILURE or EXIT_SUCCESS
   */
  int set_params(DataGeneratorParams &params);

  /*! \brief Generate the data
   *
   * Can only be run given that the parameters have ben set. Can only be run once.
   *
   * \return EXIT_FAILURE or EXIT_SUCCESS
   */
  int generate();

  /*! \brief Get const access to agents
   */
  [[nodiscard]] std::vector<Walker> Walkers() const { return walkers_; };
  /*! \brief Get const access to inter-agent loop-closures
   */
  [[nodiscard]] std::vector<MultiLoopclosure> LCs() const { return LCs_; };

  /*! \brief Save poses of agents in binary format
   *
   * Separate files for each agent and for each state (x, y, theta). Three
   * files are thus generated per agent, and named "<label><agent ID>_<state>.bin",
   * where the state is either "x", "y", or "theta". As an example, if launched
   * with default arguments, the files
   * \verbatim
     agent1_x.bin
     agent1_y.bin
     agent1_theta.bin
     \endverbatim
   * will be generated for agent 1.
   *
   * \param label The label to use for the saved files. Defaults to "agent".
   * \return EXIT_FAILURE or EXIT_SUCCESS
   */
  int save_poses_to_binary(const std::string &label = "agent");

  /*! \brief Save generated data as multi-g2o
   *
   *
   * \verbatim
     provided_path
     ├── inter_agent_lc.dat
     ├── agent1
     │   ├──  posegraph.g2o
     │   └──  agent1_GT.tum
     ├── agent2
     │   ├──  posegraph.g2o
     │   └──  ...
     ├── agent...
     │ ...
     \endverbatim
   * will be generated for agent 1.
   *
   * \param provided_path The root directory of the multi-g2o.
   * \param _3D Save in 3D representation.
   * \return EXIT_FAILURE or EXIT_SUCCESS
   */
  int save_multig2o(const std::string &provided_path, bool _3D = true);
  /*! \brief Save generated data in a single g2o file.
   *
   * This concatenates all the agents and inter-agent loop closures into a
   * single g2o-file. Not that, in order to comply with the g2o format, this
   * adds edges between the agents such that the last node of agent N, gets
   * connected to the first node of agent N+1.
   *
   * \param provided_path The directory to output the single g2o file.
   * \param _3D Save in 3D representation.
   * \return EXIT_FAILURE or EXIT_SUCCESS
   */
  int save_singleg2o(const std::string &provided_path, bool _3D = true);

  /*! \brief Save generated data to a TUM file.
   *
   * This can be used to either save ground truth, or the dead-reckoning based
   * on the odometry.
   *
   * \param provided_path The directory to output the TUM file.
   * \param ground_truth Use ground truth. If false, use the dead-reckoning.
   * \return EXIT_FAILURE or EXIT_SUCCESS
   */
  int save_singleTUM(const std::string &provided_path, bool ground_truth = true);

  friend class DataGeneratorAttorney;

private:
  bool data_has_been_generated_{false};
  bool parameters_have_been_set_{false};
  DataGeneratorParams params_;
  [[nodiscard]] bool single_agent() const { return walkers_.size() == 1; };

  std::vector<std::array<int, 2>> get_all_pairs() {
    return kollagen::get_all_pairs(static_cast<int>(walkers_.size()));
  }

  explicit DataGenerator(int number_of_walkers);
  explicit DataGenerator(const std::vector<WalkerParams> &params);
  explicit DataGenerator(int number_of_walkers, int number_of_steps);

  void add_walker();
  void add_walker(WalkerParams params);
  void add_walker(const std::vector<WalkerParams> &params);

  void step(int number_of_steps);
  void step();
  void intra_agent_loopclose(const LCParams &params);
  void intra_agent_loopclose(const std::vector<LCParams> &params);
  void inter_agent_loopclose(const InterLCParams &params);

  std::vector<int> get_walker_seeds();
  int get_max_seed();
  bool seed_already_used(int seed);
  bool different_stepsize(double stepsize);
  bool different_step_fragment(int step_fragment);

  void align_center_of_masses();
  void step_right();
  void step_left();
  void step_forward();

  [[nodiscard]] bool state_OK_for_generate() const;
  [[nodiscard]] bool state_OK_for_set_params() const;

  std::vector<MultiLoopclosure> LCs_{};
  std::vector<Walker> walkers_{};
};

inline bool DataGenerator::state_OK_for_set_params() const {
  if (data_has_been_generated_) {
    std::cerr
        << "Data has already been generated. Setting parameters not allowed."
        << '\n';
    return false;
  }
  if (parameters_have_been_set_) {
    std::cerr << "Parameters can only be set once."
              << '\n';
    return false;
  }
  return true;
}

inline int DataGenerator::set_params(DataGeneratorParams &params) {
  if (!state_OK_for_set_params()) {
    return EXIT_FAILURE;
  }
  params_ = params;
  parameters_have_been_set_ = true;
  return EXIT_SUCCESS;
};

inline bool DataGenerator::state_OK_for_generate() const {
  if (data_has_been_generated_) {
    std::cerr << "Data has already been generated." << '\n';
    return false;
  }
  if (!parameters_have_been_set_) {
    std::cerr << "Parameters have not been set. Not able to generate data."
              << '\n';
    return false;
  }
  return true;
}

inline int DataGenerator::generate() {
  if (!state_OK_for_generate()) {
    return EXIT_FAILURE;
  }

  for (int i = 0; i < params_.N_agents; ++i) {
    add_walker(params_.agent_params.at(i));
  }
  step(params_.N_steps);
  intra_agent_loopclose(params_.lc_params);

  if (params_.align_center_of_masses) {
    align_center_of_masses();
  }
  inter_agent_loopclose(params_.inter_lc_params);

  data_has_been_generated_ = true;

  return EXIT_SUCCESS;
}

inline DataGenerator::DataGenerator(DataGeneratorParams params)
    : parameters_have_been_set_(true), params_(std::move(params)) {}

inline DataGenerator::DataGenerator(int number_of_walkers) {
  for (int i = 0; i < number_of_walkers; ++i) {
    add_walker();
  }
};

inline DataGenerator::DataGenerator(int number_of_walkers,
                                    int number_of_steps) {
  for (int i = 0; i < number_of_walkers; ++i) {
    add_walker();
  }
  for (auto &walker : walkers_) {
    walker.step(number_of_steps);
  }
  data_has_been_generated_ = true;
};

inline DataGenerator::DataGenerator(const std::vector<WalkerParams> &params) {
  add_walker(params);
};

inline void DataGenerator::step(int number_of_steps) {
  for (auto &walker : walkers_) {
    walker.step(number_of_steps);
  }
}

inline void DataGenerator::step() {
  for (auto &walker : walkers_) {
    walker.step();
  }
}

inline void DataGenerator::step_right() {
  for (auto &walker : walkers_) {
    walker.step_right();
  }
}

inline void DataGenerator::step_forward() {
  for (auto &walker : walkers_) {
    walker.step_forward();
  }
}

inline void DataGenerator::step_left() {
  for (auto &walker : walkers_) {
    walker.step_left();
  }
}

inline void DataGenerator::add_walker() {
  WalkerParams params;
  add_walker(params);
}

inline void DataGenerator::add_walker(WalkerParams params) {
  params.ID = static_cast<int>(walkers_.size()) + 1;
  if (seed_already_used(params.seed)) {
    std::cerr << "[Warning] Seed of agents must not be the same. Setting to "
                 "different value."
              << '\n';
    params.seed = get_max_seed() + 1;
  }
  if (different_step_fragment(params.step_fragment)) {
    std::cerr << "[Warning] Step fragment of agents must be the same. Setting "
                 "to value of first agent."
              << '\n';
    params.step_fragment = walkers_.at(0).step_fragment();
  }
  if (different_stepsize(params.stepsize)) {
    std::cerr << "[Warning] Stepsize of agents must be the same. Setting to "
                 "value of first agent."
              << '\n';
    params.stepsize = walkers_.at(0).stepsize();
  }
  walkers_.emplace_back(Walker(params));
}

inline void DataGenerator::add_walker(const std::vector<WalkerParams> &params) {
  for (const auto &param : params) {
    add_walker(param);
  }
}

inline void DataGenerator::intra_agent_loopclose(const LCParams &params) {
  std::vector<LCParams> param_vector(walkers_.size());
  std::fill(param_vector.begin(), param_vector.end(), params);
  intra_agent_loopclose(param_vector);
}

inline void
DataGenerator::intra_agent_loopclose(const std::vector<LCParams> &params) {
  if (params.size() != walkers_.size()) {
    std::cerr
        << "Size of parameter vector doesn't match the number of parameters."
        << std::endl;
    return;
  }

  for (size_t i = 0; i < walkers_.size(); ++i) {
    if (single_agent_loopclose(walkers_.at(i), params.at(i)) != EXIT_SUCCESS) {
      std::cerr << "Walker with ID " << walkers_.at(i).ID()
                << " failed to loopclose." << std::endl;
      return;
    }
  }
}

inline void DataGenerator::inter_agent_loopclose(const InterLCParams &params) {
  if (single_agent()) {
    return;
  }
  for (const auto &pair : get_all_pairs()) {
    auto temp = kollagen::inter_agent_loopclose(walkers_.at(pair.front()),
                                        walkers_.at(pair.back()), params);
    LCs_.insert(LCs_.end(), temp.begin(), temp.end());
  }
}

inline std::vector<int> DataGenerator::get_walker_seeds() {
  std::vector<int> temp(walkers_.size());
  for (const auto &walker : walkers_) {
    temp.emplace_back(walker.seed());
  }
  return temp;
}

inline int DataGenerator::get_max_seed() {
  auto seeds = get_walker_seeds();
  auto max_it = std::max_element(seeds.begin(), seeds.end());
  if (max_it != seeds.end()) {
    return *max_it;
  }
  return 1234;
}

inline bool DataGenerator::seed_already_used(int seed) {
  auto seeds = get_walker_seeds();
  return std::find(seeds.begin(), seeds.end(), seed) != seeds.end();
}

inline bool DataGenerator::different_stepsize(double stepsize) {
  if (walkers_.empty()) {
    return false;
  }
  auto a = walkers_.front().stepsize();
  return stepsize != a;
}

inline bool DataGenerator::different_step_fragment(int step_fragment) {
  if (walkers_.empty()) {
    return false;
  }
  auto a = walkers_.front().step_fragment();
  return step_fragment != a;
}

inline int DataGenerator::save_poses_to_binary(const std::string &label) {
  if (!data_has_been_generated_) {
    std::cerr << "Data not yet generated, no poses to save." << '\n';
    return EXIT_FAILURE;
  }
  for (const auto &walker : walkers_) {
    save_binary(label + std::to_string(walker.ID()) + "_x.bin", walker.X());
    save_binary(label + std::to_string(walker.ID()) + "_y.bin", walker.Y());
    save_binary(label + std::to_string(walker.ID()) + "_theta.bin",
                walker.Theta());
  }
  return EXIT_SUCCESS;
}

inline std::filesystem::path get_path(const std::string &provided_path) {
  return {provided_path};
};

inline std::filesystem::path
create_base_directory(const std::string &provided_path) {
  if (provided_path == "") {
    return std::filesystem::path("");
  }
  auto path = get_path(provided_path);
  std::filesystem::create_directories(path);
  return path;
};

inline int DataGenerator::save_multig2o(const std::string &provided_path,
                                        bool _3D) {
  if (!data_has_been_generated_) {
    std::cerr << "Data not yet generated, nothing to save." << '\n';
    return EXIT_FAILURE;
  }

  const auto path = create_base_directory(provided_path);

  for (int i = 0; auto &walker : walkers_) {
    std::string walker_name = "agent" + std::to_string(i + 1);
    std::filesystem::path walker_path(path / walker_name);

    std::filesystem::create_directory(walker_path);

    walker.writeg2o((walker_path / "posegraph.g2o").string(), _3D);
    walker.write_ground_truth_as_TUM((walker_path / (walker_name + "_GT.tum")).string());

    i++;
  }
  int OK = write_inter_agent_LC_to_dat(LCs_, (path / "inter_agent_lc.dat").string(), _3D);

  return OK;
}

inline int DataGenerator::save_singleTUM(const std::string &provided_path, bool ground_truth) {
  if (!data_has_been_generated_) {
    std::cerr << "Data not yet generated, nothing to save." << '\n';
    return EXIT_FAILURE;
  }

  const auto path = create_base_directory(std::filesystem::path(provided_path).parent_path().string());

  auto file = open_file(provided_path, ".tum");

  int offset = 0;
  for (const auto &walker : walkers_) {
    ground_truth ? write_TUM_vertices(file, {walker.X(), walker.Y(), walker.Theta()}, offset) :
                   write_TUM_vertices(file, {walker.Drx(), walker.Dry(), walker.Drtheta()}, offset);
    offset += static_cast<int>(walker.size());
  }

  return EXIT_SUCCESS;
}

inline int DataGenerator::save_singleg2o(const std::string &provided_path,
                                         bool _3D) {
  if (!data_has_been_generated_) {
    std::cerr << "Data not yet generated, nothing to save." << '\n';
    return EXIT_FAILURE;
  }

  const auto path = create_base_directory(std::filesystem::path(provided_path).parent_path().string());

  auto file = open_file(provided_path, ".g2o");

  std::vector<int> offset{0};
  std::map<WalkerID, int> ID_to_index_map{};

  for (int i = 0; const auto &walker : walkers_) {
    write_vertices(file, {walker.Drx(), walker.Dry(), walker.Drtheta()}, _3D,
                   offset.at(i));
    offset.emplace_back(static_cast<int>(walker.size()) + offset.at(i));
    ID_to_index_map.insert({walker.ID(), i});
    i++;
  }

  double x_prev{};
  double y_prev{};
  double theta_prev{};
  for (int i = 0; const auto &walker : walkers_) {
    if (i>0) {
      // create the connection between previous walker and current
      auto [dx, dy, dtheta] = get_relative_transform_from_pointA_to_pointB({x_prev,y_prev,theta_prev},{walker.X().front(),walker.Y().front(),walker.Theta().front()});
      int from = offset.at(i)-1;
      int to = offset.at(i);
      double I11 = params_.inter_lc_params.I11est;
      double I22 = params_.inter_lc_params.I22est;
      double I33 = params_.inter_lc_params.I33est;
      if (!_3D) {
        write_2D_edge_to_file(file, from, to, dx, dy ,dtheta, I11, I22, I33);
      } else if (_3D) {
        write_3D_edge_to_file(file, from, to, dx, dy ,dtheta, I11, I22, I33);
      }
    }
    write_odometry(file, walker, _3D, params_.exact_information_matrix, offset.at(i));
    x_prev = walker.X().back();
    y_prev = walker.Y().back();
    theta_prev = walker.Theta().back();
    i++;
  }

  for (int i = 0; const auto &walker : walkers_) {
    write_loop_closures(file, walker.LC(), _3D, offset.at(i));
    i++;
  }

  write_inter_agent_loop_closures(file, LCs_, _3D, offset, ID_to_index_map);

  return EXIT_SUCCESS;
}

inline void DataGenerator::align_center_of_masses() {
  auto [x, y] = center_of_mass(walkers_.at(0));
  int x_int = round_to_int(x);
  int y_int = round_to_int(y);

  for (auto &walker : walkers_) {
    auto [x2, y2] = center_of_mass(walker);
    int x_int2 = round_to_int(x2);
    int y_int2 = round_to_int(y2);

    int x_diff = x_int - x_int2;
    int y_diff = y_int - y_int2;
    if (!(x_diff == 0 && y_diff == 0)) {
      walker.translate(x_diff, y_diff);
    }
  }
}

}  // namespace kollagen
#endif /* !DATAGENERATOR_H */
