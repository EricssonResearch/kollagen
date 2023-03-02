#ifndef MULTIG2O_PARSE_H
#define MULTIG2O_PARSE_H

#include <string>
#include <tuple>
#include <stdexcept>
#include <algorithm>
#include <memory>
#include <iostream>
#include <fstream>
#include <utility>
#include <vector>
#include <filesystem>

namespace kollagen::multig2o
{

struct Vertex {
  Vertex(int ID, std::vector<double> pose) : ID(ID), pose(std::move(pose)) {};
  int ID;
  std::vector<double> pose{};
};

struct Edge {
  Edge(int from, int to, std::vector<double> transform, std::vector<double> information_matrix) :
    from(from),
    to(to),
    transform(std::move(transform)),
    information_matrix(std::move(information_matrix)){};
  int from{};
  int to{};
  std::vector<double> transform{};
  std::vector<double> information_matrix{};
};

struct InterAgentLC {
  InterAgentLC(int from_agent, int from_index, int to_agent, int to_index, std::vector<double> transform, std::vector<double> information_matrix) :
    from_agent(from_agent),
    from_index(from_index),
    to_agent(to_agent),
    to_index(to_index),
    edge{from_index, to_index, std::move(transform), std::move(information_matrix)}{};
  int from_agent{};
  int from_index{};
  int to_agent{};
  int to_index{};
  Edge edge;
};

struct AgentEntries {
  std::vector<Vertex> vertices{};
  std::vector<Edge> odometry{};
  std::vector<Edge> loop_closures{};
  std::vector<std::array<double, 8>> ground_truth{};
};

struct MultiG2OPaths{
  std::vector<std::filesystem::path> g2o_files{};
  std::vector<std::filesystem::path> ground_truth_files{};
  std::filesystem::path inter_agent_lc{};
};

struct MultiG2O
{
  std::vector<AgentEntries> agents{};
  std::vector<InterAgentLC> inter_agent_lcs{};
};

namespace helper
{

inline bool is_agent_directory(const std::filesystem::directory_entry& dir_entry, const std::string& agent_dir_prefix = "agent"){
    std::string dir_entry_string = dir_entry.path().filename().string();
    bool result = false;
    bool has_agent_prefix = dir_entry_string.compare(0, agent_dir_prefix.size(), agent_dir_prefix) == 0;

    if (!has_agent_prefix) {
      return false;
    }

    bool contains_g2o = std::filesystem::exists(dir_entry.path()/"posegraph.g2o");
    bool contains_tum = std::filesystem::exists(dir_entry.path()/(dir_entry_string+"_GT.tum"));
    result = contains_g2o && contains_tum;
    return result;
}

inline std::vector<std::filesystem::path> get_sorted_agent_dirs(const std::filesystem::path& dir){
  using std::filesystem::directory_iterator;
  std::vector<std::filesystem::path> agent_dirs{};

  for (const auto& dir_entry : directory_iterator(dir)) {
    if (is_agent_directory(dir_entry)) {
      agent_dirs.emplace_back(dir_entry.path());
    }
  }
  std::sort(agent_dirs.begin(), agent_dirs.end());

  return agent_dirs;
}

inline MultiG2OPaths extract_paths(const std::filesystem::path& dir){
  std::filesystem::path inter_agent_lc = dir/"inter_agent_lc.dat";
  if (!std::filesystem::exists(inter_agent_lc)) {
    throw std::invalid_argument("multig2o is missing an inter_agent_lc.dat");
  }

  auto agent_dirs = get_sorted_agent_dirs(dir);
  if (agent_dirs.empty()) {
    throw std::invalid_argument("Unable to find agent folders in provided multig2o");
  }

  MultiG2OPaths paths{};
  for (const auto& agent_dir : agent_dirs) {
    paths.g2o_files.emplace_back(agent_dir/"posegraph.g2o");
    paths.ground_truth_files.emplace_back(agent_dir/(agent_dir.filename().string()+"_GT.tum"));
  }
  paths.inter_agent_lc = inter_agent_lc;

  return paths;
}

inline Vertex parse_vertex(std::ifstream& g2o, const bool& _3D){
  if (_3D) {
    int ID;
    double x, y, z, qx, qy, qz, qw;
    g2o >> ID >> x >> y >> z >> qx >> qy >> qz >> qw;
    return {ID, {x, y, z, qx, qy, qz, qw}};
  }
  int ID;
  double x, y, theta;
  g2o >> ID >> x >> y >> theta;
  return {ID, {x, y, theta}};
}

inline bool is_loopclosure(int from, int to){
  return to - from != 1;
}

inline std::vector<InterAgentLC> parse_inter_agent_lc(std::ifstream& inter_agent_lc, const bool& _3D){
  std::vector<InterAgentLC> result{};
  if (_3D) {
    int from_agent;
    int from_index;
    int to_agent;
    int to_index;
    double x, y, z, qx, qy, qz, qw;
    double I11, I12, I13, I14, I15, I16,
                I22, I23, I24, I25, I26,
                     I33, I34, I35, I36,
                          I44, I45, I46,
                               I55, I56,
                                    I66;
    while (
    inter_agent_lc >> from_agent >> from_index >> to_agent >> to_index >> x >> y >> z >> qx >> qy >> qz >> qw >>
           I11 >> I12 >> I13 >> I14 >> I15 >> I16 >>
                  I22 >> I23 >> I24 >> I25 >> I26 >>
                         I33 >> I34 >> I35 >> I36 >>
                                I44 >> I45 >> I46 >>
                                       I55 >> I56 >>
                                              I66) {
      result.emplace_back(InterAgentLC{from_agent, from_index, to_agent, to_index, {x, y, z, qx, qy, qz, qw}, {I11, I12, I13, I14, I15, I16,
      I22, I23, I24, I25, I26, I33, I34, I35, I36, I44, I45, I46, I55, I56, I66}});
    }
    return result;
  }
  int from_agent;
  int from_index;
  int to_agent;
  int to_index;
  double x, y, theta;
  double I11, I12, I13, I22, I23, I33;
  while (
  inter_agent_lc >> from_agent >> from_index >> to_agent >> to_index >> x >> y >> theta >>
         I11 >> I12 >> I13 >>
                I22 >> I23 >>
                       I33) {

  result.emplace_back(InterAgentLC{from_agent, from_index, to_agent, to_index, {x, y, theta}, {I11, I12, I13, I22, I23, I33}});
  }
  return result;
}

inline std::tuple<Edge, bool> parse_edge(std::ifstream& g2o, const bool& _3D){
  if (_3D) {
    int from;
    int to;
    double x, y, z, qx, qy, qz, qw;
    double I11, I12, I13, I14, I15, I16,
                I22, I23, I24, I25, I26,
                     I33, I34, I35, I36,
                          I44, I45, I46,
                               I55, I56,
                                    I66;
    g2o >> from >> to >> x >> y >> z >> qx >> qy >> qz >> qw >>
           I11 >> I12 >> I13 >> I14 >> I15 >> I16 >>
                  I22 >> I23 >> I24 >> I25 >> I26 >>
                         I33 >> I34 >> I35 >> I36 >>
                                I44 >> I45 >> I46 >>
                                       I55 >> I56 >>
                                              I66;
    return {{from, to, {x, y, z, qx, qy, qz, qw}, {I11, I12, I13, I14, I15, I16,
      I22, I23, I24, I25, I26, I33, I34, I35, I36, I44, I45, I46, I55, I56, I66
    }}, is_loopclosure(from, to)};
  }
  int from;
  int to;
  double x, y, theta;
  double I11, I12, I13, I22, I23, I33;
  g2o >> from >> to  >> x >> y >> theta >>
         I11 >> I12 >> I13 >>
                I22 >> I23 >>
                       I33;
  return {{from, to , {x, y, theta}, {I11, I12, I13, I22, I23, I33}}, is_loopclosure(from, to)};
}

inline std::tuple<std::vector<Vertex>, std::vector<Edge>, std::vector<Edge>> parse_g2o(
    const std::filesystem::path& g2o_path, const bool& _3D
    ){
  std::ifstream g2o(g2o_path.c_str());
  if (!g2o) {
    throw std::invalid_argument("file not found");
  }
  std::string type;
  std::string vertex_type = _3D ? "VERTEX_SE3:QUAT" : "VERTEX_SE2";
  std::string edge_type = _3D ? "EDGE_SE3:QUAT" : "EDGE_SE2";

  std::vector<Vertex> vertices{};
  std::vector<Edge> odometry{};
  std::vector<Edge> loop_closures{};

  g2o >> type;
  if (type != vertex_type) {
    throw std::invalid_argument("First line of g2o should read " + vertex_type);
  }

  do {
    if (type == edge_type) {
      break;
    }
    vertices.emplace_back(parse_vertex(g2o, _3D));

  } while (g2o >> type);

  if (type != edge_type) {
    throw std::invalid_argument("First line after " + vertex_type + " in g2o should read " + edge_type);
  }

  do {
    auto [edge, is_loopclosure] = parse_edge(g2o, _3D);

    if (is_loopclosure) {
      loop_closures.emplace_back(edge);
      break;
    }
    odometry.emplace_back(edge);

  } while (g2o >> type);

  while (g2o >> type){

    auto [edge, is_loopclosure] = parse_edge(g2o, _3D);
    loop_closures.emplace_back(edge);

  }

  return {vertices, odometry, loop_closures};
}

inline std::vector<std::array<double, 8>> parse_TUM(const std::filesystem::path& tum_path){
  std::vector<std::array<double, 8>> result;
  std::ifstream gt(tum_path.c_str());
  if (!gt) {
    throw std::invalid_argument("file not found");
  }
  double timestamp, tx, ty, tz, qx, qy, qz, qw;
  while(gt >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw){
    result.emplace_back(std::array<double,8>({timestamp, tx, ty, tz, qx, qy, qz, qw}));
  }
  return result;
}

inline AgentEntries get_agent_entries(const std::filesystem::path& g2o_path, const std::filesystem::path& gt_path, const bool& _3D){
  AgentEntries result;

  result.ground_truth = parse_TUM(gt_path);
  std::tie(result.vertices, result.odometry, result.loop_closures) = parse_g2o(g2o_path, _3D);

  return result;
}

inline std::vector<InterAgentLC> get_inter_agent_lcs(const std::filesystem::path& inter_agent_lc_path, const bool& _3D){
  std::vector<InterAgentLC> result{};

  std::ifstream inter_agent_lc(inter_agent_lc_path.c_str());
  if (!inter_agent_lc) {
    throw std::invalid_argument("file not found");
  }

  result = parse_inter_agent_lc(inter_agent_lc, _3D);

  return result;
}


inline MultiG2O parse(const MultiG2OPaths& paths, const bool& _3D){
  MultiG2O multi_g2o{};
  for (int i = 0; i < static_cast<int>(paths.g2o_files.size()); ++i) {
    auto g2o_path = paths.g2o_files.at(i);
    auto gt_path = paths.ground_truth_files.at(i);
    multi_g2o.agents.emplace_back(get_agent_entries(g2o_path, gt_path, _3D));
  }
  multi_g2o.inter_agent_lcs = get_inter_agent_lcs(paths.inter_agent_lc, _3D);
  return {};
}

}  // namespace helper

inline MultiG2O open(const std::string& dirname, const bool& _3D = true){
  std::filesystem::path path(dirname);
  MultiG2OPaths paths = helper::extract_paths(path);
  MultiG2O data = helper::parse(paths, _3D);
  return data;
}

}  // namespace kollagen::multig2o
#endif /* !MULTIG2O_PARSE_H */
