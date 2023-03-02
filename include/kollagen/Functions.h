#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "Walker.h"

namespace kollagen
{

using VisitMap = std::map<std::pair<int, int>, int>;

inline std::ofstream open_file(const std::string &filename, const std::string& forced_extension = "") {
  std::filesystem::path path(filename);
  if (path.extension() != forced_extension) {
    path += forced_extension;
  }
  std::cout << "Saving to " << path.string() << '\n';

  std::ofstream file(path.string(), std::ios_base::out);

  return file;
}

inline void write_TUM_vertices(std::ofstream &file,
                           const std::array<std::vector<double>, 3> &XYTheta,
                           const int &offset = 0){
  auto [X, Y, Theta] = XYTheta;
  for (int i = 0; i < static_cast<int>(X.size()); ++i) {
    double x = X.at(i);
    double y = Y.at(i);
    double theta = Theta.at(i);
    auto timestamp = static_cast<double>(i + offset);
    file << string_format("%.17g %.17g %.17g 0.0 0.0 0.0 %.17g %.17g\n",
                          timestamp, x, y, std::sin(theta / 2),
                          std::cos(theta / 2));
  }
}

inline void write_vertices(std::ofstream &file,
                           const std::array<std::vector<double>, 3> &XYTheta,
                           const bool _3D, const int &offset = 0) {
  auto [X, Y, Theta] = XYTheta;
  for (int i = 0; i < static_cast<int>(X.size()); ++i) {
    double x = X.at(i);
    double y = Y.at(i);
    double theta = Theta.at(i);
    if (!_3D) {
      file << string_format("VERTEX_SE2 %i %.17g %.17g %.17g\n", i + offset, x,
                            y, theta);
    } else if (_3D) {
      file << string_format(
          "VERTEX_SE3:QUAT %i %.17g %.17g 0.0 0.0 0.0 %.17g %.17g\n",
          i + offset, x, y, std::sin(theta / 2), std::cos(theta / 2));
    }
  }
}

inline std::array<double, 3>
get_relative_transform_from_pointA_to_pointB(std::array<double, 3> pointA,
                                             std::array<double, 3> pointB) {
  auto [x1, y1, theta1] = pointA;
  auto [x2, y2, theta2] = pointB;
  double dtheta = normalize_angle(theta2 - theta1);
  double dx = (x2 - x1);
  double dy = (y2 - y1);

  double dx_rel = dx*std::cos(theta1) + dy*std::sin(theta1);
  double dy_rel =-dx*std::sin(theta1) + dy*std::cos(theta1);
  double dtheta_rel = dtheta;

  return {dx_rel, dy_rel, dtheta_rel};
}

inline std::array<double, 3>
odometry_to_body_frame_transformation(const double &odom_pos,
                                      const double &odom_ang) {
  double dtheta = odom_ang;
  double dx = odom_pos * std::cos(dtheta);
  double dy = odom_pos * std::sin(dtheta);
  return {dx, dy, dtheta};
}

inline void write_2D_edge_to_file(std::ofstream &file, const int &from,
                                  const int &to, const double &dx,
                                  const double &dy, const double &dtheta,
                                  const double &I11, const double &I22,
                                  const double &I33,
                                  const double &I12 = 0.0,
                                  const double &I13 = 0.0,
                                  const double &I23 = 0.0
                                  ) {
  file << string_format("EDGE_SE2 %i %i %.17g %.17g %.17g %.17g %.17g "
                        "%.17g %.17g %.17g %.17g\n",
                        from, to, dx, dy, dtheta, I11, I12, I13, I22, I23, I33);
}
inline void write_3D_edge_to_file(std::ofstream &file, const int &from,
                                  const int &to, const double &dx,
                                  const double &dy, const double &dtheta,
                                  const double &I11, const double &I22,
                                  const double &I33,
                                  const double &I12 = 0.0,
                                  const double &I13 = 0.0,
                                  const double &I23 = 0.0
                                  ) {
  double quaternion_z = std::sin(dtheta / 2);
  double quaternion_w = std::cos(dtheta / 2);
  file << string_format("EDGE_SE3:QUAT %i %i %.17g %.17g 0.0 0.0 0.0 %.17g "
                        "%.17g %.17g 0.0 0.0 0.0 0.0 0.0 %.17g 0.0 0.0 0.0 0.0 "
                        "%.17g 0.0 0.0 0.0 %.17g 0.0 0.0 %.17g 0.0 %.17g\n",
                        from, to, dx, dy, quaternion_z, quaternion_w, I11, I22,
                        1e6, I33, I33, I33, I12, I13, I23);
}

inline void set_information_matrix(double& I11, double& I22, double& I33, double& I12, double& I13, double& I23, const Walker &walker, const int& i){
  // See doc/information-matrix-derivation.pdf
  double d_ij = walker.stepsize()/walker.step_fragment();
  double sigma_d = walker.std_dev_pos();
  double sigma_theta = walker.std_dev_ang();

  double d_ij_squared = std::pow(d_ij, 2);
  double sigma_d_squared = std::pow(sigma_d, 2);
  double sigma_theta_squared = std::pow(sigma_theta, 2);

  double expression_1 = (d_ij_squared + sigma_d_squared) / 2.0 * (1 - std::exp(-2*sigma_theta_squared));
  double expression_2 = (d_ij_squared + sigma_d_squared) / 2.0 * (1 + std::exp(-2*sigma_theta_squared)) - d_ij_squared * std::exp(-sigma_theta_squared);

  double a = expression_1 * sigma_theta_squared - d_ij_squared * std::pow(sigma_theta, 4)*std::exp(-sigma_theta_squared);
  double b = a;
  double c = d_ij*sigma_theta_squared*std::exp(-0.5*sigma_theta_squared);

  double dtheta = walker.dtheta(i);

  if (dtheta == 0 || dtheta == 2) {
      auto sigma_x_squared = expression_2;
      auto sigma_y_squared = expression_1;
      I11 = 1.0/sigma_x_squared;
      I22 = sigma_theta_squared/a;
      I33 = sigma_y_squared/a;
      I12 = 0.0;
      I13 = 0.0;
      I23 = c/a;
      if (dtheta == 0) {
        I23 *= -1;
      }
  } else if (dtheta == 1 || dtheta == -1) {
      auto sigma_x_squared = expression_1;
      auto sigma_y_squared = expression_2;
      I11 = sigma_theta_squared/b;
      I22 = 1.0/sigma_y_squared;
      I33 = sigma_x_squared/b;
      I12 = 0.0;
      I13 = c/b;
      I23 = 0.0;
      if (dtheta == -1) {
        I13 *= -1;
      }
  }
}

inline void write_odometry(std::ofstream &file,
                           const Walker &walker,
                           const bool _3D,
                           const bool &exact_information_matrix = false,
                           const int &offset = 0
                           ) {
  auto odom_pos = walker.odom_pos();
  auto odom_ang = walker.odom_ang();
  auto I11est = walker.Params().I11est;
  auto I22est = walker.Params().I22est;
  auto I33est = walker.Params().I33est;
  double I12est = 0.0;
  double I13est = 0.0;
  double I23est = 0.0;

  for (int i = 0; i < static_cast<int>(odom_ang.size()); ++i) {
    if (exact_information_matrix) {
      set_information_matrix(I11est, I22est, I33est, I12est, I13est, I23est, walker, i);
    }

    auto [dx, dy, dtheta] =
        odometry_to_body_frame_transformation(odom_pos.at(i), odom_ang.at(i));

    if (!_3D) {
      write_2D_edge_to_file(file, i + offset, i + offset + 1, dx, dy, dtheta,
                            I11est, I22est, I33est, I12est, I13est, I23est);
    } else if (_3D) {
      write_3D_edge_to_file(file, i + offset, i + offset + 1, dx, dy, dtheta,
                            I11est, I22est, I33est, I12est, I13est, I23est);
    }
  }
}

inline void write_loop_closures(std::ofstream &file,
                                const std::vector<Loopclosure> &LCs,
                                const bool _3D, const int &offset = 0) {
  for (const auto &lc : LCs) {
    if (!_3D) {
      write_2D_edge_to_file(file, lc.from + offset, lc.to + offset, lc.dx,
                            lc.dy, lc.dtheta, lc.I11, lc.I22, lc.I33);
    } else if (_3D) {
      write_3D_edge_to_file(file, lc.from + offset, lc.to + offset, lc.dx,
                            lc.dy, lc.dtheta, lc.I11, lc.I22, lc.I33);
    }
  }
}

inline void write_inter_agent_loop_closures(
    std::ofstream &file, const std::vector<MultiLoopclosure> &LCs,
    const bool _3D, const std::vector<int> &offset,
    const std::map<WalkerID, int>& ID_to_index_map
    ) {
  for (const auto &lc : LCs) {
    auto offset_index_1 = ID_to_index_map.at(lc.ID1);
    auto offset_index_2 = ID_to_index_map.at(lc.ID2);
    if (!_3D) {
      write_2D_edge_to_file(file, lc.from + offset.at(offset_index_1),
                            lc.to + offset.at(offset_index_2), lc.dx, lc.dy, lc.dtheta,
                            lc.I11, lc.I22, lc.I33);
    } else if (_3D) {
      write_3D_edge_to_file(file, lc.from + offset.at(offset_index_1),
                            lc.to + offset.at(offset_index_2), lc.dx, lc.dy, lc.dtheta,
                            lc.I11, lc.I22, lc.I33);
    }
  }
}

inline std::tuple<int, int> get_int_xy_at(const Walker &walker, int i) {
  int x_int =
      round_to_int(walker.X(i) * walker.step_fragment() / walker.stepsize());
  int y_int =
      round_to_int(walker.Y(i) * walker.step_fragment() / walker.stepsize());
  return {x_int, y_int};
}

inline int get_previous_visit_index(const VisitMap &visited, const int &x_int,
                                    const int &y_int) {
  auto iterator = visited.find(std::make_pair(x_int, y_int));
  if (iterator != visited.end()) {
    return iterator->second;
  }
  return -1;
}

inline std::tuple<double, double, double>
calculate_relative_transform(const Walker &walker, int i, int j) {
  double xdiff = walker.X(i) - walker.X(j);
  double ydiff = walker.Y(i) - walker.Y(j);
  double thetadiff = walker.Theta(i) - walker.Theta(j);

  double DX =
      xdiff * std::cos(walker.Theta(j)) + ydiff * std::sin(walker.Theta(j));
  double DY =
      -xdiff * std::sin(walker.Theta(j)) + ydiff * std::cos(walker.Theta(j));
  double DTheta = normalize_angle(thetadiff);

  return {DX, DY, DTheta};
}

inline std::tuple<double, double, double>
calculate_relative_transform(const Walker &walker, const Walker &walker2,
                             int i1, int i2) {
  double xdiff = walker.X(i1) - walker2.X(i2);
  double ydiff = walker.Y(i1) - walker2.Y(i2);
  double thetadiff = walker.Theta(i1) - walker2.Theta(i2);

  double DTheta = normalize_angle(thetadiff);
  double DX =
      xdiff * std::cos(walker2.Theta(i2)) + ydiff * std::sin(walker2.Theta(i2));
  double DY = -xdiff * std::sin(walker2.Theta(i2)) +
              ydiff * std::cos(walker2.Theta(i2));

  return {DX, DY, DTheta};
}

inline int
single_agent_loopclose(Walker &walker, const LCParams &param,
                       std::function<double(Point, LCParams)> norm_function) {

  VisitMap visited{};

  for (int i = 0; i < static_cast<int>(walker.size()); ++i) {
    int previous_visit_index{-1};

    auto [x_int, y_int] = get_int_xy_at(walker, i);

    previous_visit_index = get_previous_visit_index(visited, x_int, y_int);

    if (previous_visit_index >= 0) {
      if (param.idemLCprob > walker.draw_uniform()) {
        int j = previous_visit_index;
        auto [DX, DY, DTheta] = calculate_relative_transform(walker, i, j);
        walker.addLC(
            Loopclosure(j, i, DX + walker.draw_normal() * param.sigma_pos,
                        DY + walker.draw_normal() * param.sigma_pos,
                        DTheta + walker.draw_normal() * param.sigma_ang,
                        param.I11est, param.I22est, param.I33est));
      }
    } else {
      visited[std::make_pair(x_int, y_int)] = i;
    }

    const int integer_radius =
        round_to_int(param.radius * walker.step_fragment() / walker.stepsize());

    auto circle = DiscreteCircle(integer_radius);

    for (const auto &coordinate : circle) {
      bool at_current_position = coordinate.x == 0 && coordinate.y == 0;
      bool non_diagonal = coordinate.x == 0 || coordinate.y == 0;

      if (at_current_position) {
        continue;
      }
      if (non_diagonal) {
        continue;
      }

      previous_visit_index = get_previous_visit_index(
          visited, x_int + coordinate.x, y_int + coordinate.y);

      if (previous_visit_index >= 0) {
        double prob = param.prob_scale * norm_function(coordinate, param);
        if (prob > walker.draw_uniform()) {
          int j = previous_visit_index;
          auto [DX, DY, DTheta] = calculate_relative_transform(walker, i, j);
          walker.addLC(
              Loopclosure(j, i, DX + walker.draw_normal() * param.sigma_pos,
                          DY + walker.draw_normal() * param.sigma_pos,
                          DTheta + walker.draw_normal() * param.sigma_ang,
                          param.I11est, param.I22est, param.I33est));
        }
      }
    }
  }

  return EXIT_SUCCESS;
}

inline int single_agent_loopclose(Walker &walker, const LCParams &param) {
  return single_agent_loopclose(walker, param, [](Point p, LCParams param) {
    return std::exp(-5.0 * (p.x * p.x + p.y * p.y) /
                    (param.radius * param.radius));
  });
  ;
}

inline std::vector<MultiLoopclosure> inter_agent_loopclose(
    Walker &walker, Walker &walker2, const InterLCParams &param,
    std::function<double(Point, InterLCParams)> norm_function) {

  RNGenerator rng(param.seed);

  const int integer_radius =
      round_to_int(param.radius * walker.step_fragment() / walker.stepsize());

  auto circle = DiscreteCircle(integer_radius);

  VisitMap visited{};

  std::vector<MultiLoopclosure> LCs{};

  for (int i = 0; i < static_cast<int>(walker.size()); ++i) {
    auto [x_int, y_int] = get_int_xy_at(walker, i);
    visited[std::make_pair(x_int, y_int)] = i;
  }

  for (int i = 0; i < static_cast<int>(walker2.size()); ++i) {

    auto [x_int2, y_int2] = get_int_xy_at(walker2, i);

    int previous_visit = get_previous_visit_index(visited, x_int2, y_int2);

    if (previous_visit >= 0) {
      int j = previous_visit;
      auto [DX, DY, DTheta] =
          calculate_relative_transform(walker, walker2, j, i);

      if (param.idemLCprob > rng.draw_uniform()) {
        LCs.emplace_back(
            MultiLoopclosure(walker2.ID(), walker.ID(), i, j,
                             DX + rng.draw_normal() * param.sigma_pos,
                             DY + rng.draw_normal() * param.sigma_pos,
                             DTheta + rng.draw_normal() * param.sigma_ang,
                             param.I11est, param.I22est, param.I33est));
      }
    }

    for (const auto &coordinate : circle) {
      bool at_current_position = coordinate.x == 0 && coordinate.y == 0;
      if (at_current_position) {
        continue;
      }

      previous_visit = get_previous_visit_index(visited, x_int2 + coordinate.x,
                                                y_int2 + coordinate.y);

      if (previous_visit >= 0) {
        double prob = param.prob_scale * norm_function(coordinate, param);
        if (prob > rng.draw_uniform()) {
          int j = previous_visit;
          auto [DX, DY, DTheta] =
              calculate_relative_transform(walker, walker2, j, i);

          LCs.emplace_back(
              MultiLoopclosure(walker2.ID(), walker.ID(), i, j,
                               DX + rng.draw_normal() * param.sigma_pos,
                               DY + rng.draw_normal() * param.sigma_pos,
                               DTheta + rng.draw_normal() * param.sigma_ang,
                               param.I11est, param.I22est, param.I33est));
        }
      }
    }
  }

  return LCs;
};

inline std::vector<MultiLoopclosure>
inter_agent_loopclose(Walker &walker, Walker &walker2,
                      const InterLCParams &param) {
  return inter_agent_loopclose(
      walker, walker2, param, [](Point p, InterLCParams param) {
        return std::exp(-5.0 * (p.x * p.x + p.y * p.y) /
                        (param.radius * param.radius));
      });
};

inline int write_inter_agent_LC_to_dat(const std::vector<MultiLoopclosure> &LCs,
                                       const std::string &filename,
                                       const bool &_3D = false) {
  std::filesystem::path path(filename);
  if (path.extension() != ".dat") {
    path += ".dat";
  }
  std::cout << "Saving to " << path.string() << '\n';

  std::ofstream file(path.string(), std::ios_base::out);
  if (!file) {
    return EXIT_FAILURE;
  }
  for (const auto &lc : LCs) {
    if (!_3D) {
      file << string_format("%i %i %i %i %f %f %f %f 0.0 "
                            "0.0 %f 0.0 %f\n",
                            lc.ID1, lc.from, lc.ID2, lc.to, lc.dx, lc.dy,
                            lc.dtheta, lc.I11, lc.I22, lc.I33);
    } else if (_3D) {
      file << string_format(
          "%i %i %i %i %f %f 0.0 0.0 0.0 %f %f %f 0.0 0.0 0.0 0.0 0.0 %f 0.0 "
          "0.0 0.0 0.0 %f 0.0 0.0 0.0 %f 0.0 0.0 %f 0.0 %f\n",
          lc.ID1, lc.from, lc.ID2, lc.to, lc.dx, lc.dy, std::sin(lc.dtheta / 2),
          std::cos(lc.dtheta / 2), lc.I11, lc.I22, 1e6, lc.I33, lc.I33, lc.I33);
    }
  }

  return EXIT_SUCCESS;
};

inline std::vector<std::array<int, 2>> get_all_pairs(int N) {
  // This function returns the indexes to all the N choose 2 possible pairs
  // where ordering is irrelevant. (That is {1,2} is equivalent to {2,1})
  std::vector<std::array<int, 2>> result{};
  std::string bitmask(2, 1);
  bitmask.resize(N, 0);

  do {
    int index = 0;
    std::array<int, 2> temp{};
    for (int i = 0; i < N; ++i) {
      if (bitmask[i] != 0) {
        temp.at(index) = i;
        index += 1;
      }
    }
    result.emplace_back(temp);
  } while (std::prev_permutation(bitmask.begin(), bitmask.end()));

  return result;
}

inline std::tuple<double, double> center_of_mass(const Walker &walker) {
  int N = static_cast<int>(walker.size());
  auto X = walker.X();
  auto Y = walker.Y();
  double x = std::reduce(X.begin(), X.end()) / N;
  double y = std::reduce(Y.begin(), Y.end()) / N;
  return {x, y};
}

}  // namespace kollagen
#endif /* !FUNCTIONS_H */
