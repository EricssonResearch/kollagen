#include <filesystem>
#include <iostream>
#include <tuple>

#include "kollagen.h"

namespace kollagen
{

void print_help();

std::tuple<std::filesystem::path, std::filesystem::path>
get_CLI_arguments(int argc, char *argv[]);

void add_extension(std::filesystem::path json_file);

bool CLI_arguments_OK(const std::filesystem::path &json_file,
                      const std::filesystem::path &save_dir,
                      const bool &multi_g2o,
                      const bool &single_g2o
                      );

void generate_and_save(const std::filesystem::path &json_file,
                       const std::filesystem::path &save_dir, const bool &multig2o,
                       const bool &singleg2o);

inline void print_help() {
  std::cout << "NAME\n"
               "   generate - generate data from json file\n\n"
               "SYNOPSIS\n"
               "       generate SOURCE [DEST]\n\n"
               "DESCRIPTION\n"
               "       Generate data from json file SOURCE. Save in folder "
               "DEST, defaulting to './output'.\n";
}

inline void add_extension(std::filesystem::path& file, const std::string& extension) {
  if (file.extension() != extension) {
   file.replace_extension(extension);
  }
}

inline std::tuple<std::filesystem::path, std::filesystem::path>
get_CLI_arguments(int argc, char *argv[]) {
  std::filesystem::path json_file{};
  std::filesystem::path save_dir{};

  if (argc == 1) {
    print_help();
    return {"", ""};
  }

  if (argc > 1) {
    json_file = argv[1];
    save_dir = "./output";
    if (json_file == "-h" || json_file == "--help") {
      print_help();
      return {"", ""};
    }
    add_extension(json_file, "json");
  }

  if (argc > 2) {
    save_dir = argv[2];
  }

  return {json_file, save_dir};
}

inline bool CLI_arguments_OK(const std::filesystem::path &json_file,
                      const std::filesystem::path &save_dir,
                      const bool& multig2o,
                      const bool& singleg2o
                      ) {
  bool no_files_to_save = ! (multig2o || singleg2o);
  bool both_paths_empty = json_file.empty() && save_dir.empty();
  bool no_such_json_file = !std::filesystem::exists(json_file);

  if (no_files_to_save) {
    std::cout << "[Error]: No files to save. Exiting." << '\n';
    return false;
  }
  if (both_paths_empty) {
    std::cout << "[Error]: No paths given." << '\n';
    return false;
  }
  if (no_such_json_file) {
    std::cout << "[Error] " + json_file.string() + ": No such file." << '\n';
    return false;
  }
  return true;
};

inline void generate_and_save(const std::filesystem::path &json_file,
                       const std::filesystem::path &save_dir, const bool &multig2o,
                       const bool &singleg2o) {
  auto params = get_params_from_json(json_file.string());

  auto data_gen = DataGenerator(params);
  data_gen.generate();

  if (multig2o) {
    data_gen.save_multig2o((save_dir / json_file.stem()).string());
  }

  if (singleg2o) {
    auto singleg2o_save_dir = save_dir / (json_file.stem().string() + "_singleg2o");
    data_gen.save_singleg2o((singleg2o_save_dir / json_file.stem()).string() + ".g2o",
                            false);
    data_gen.save_singleTUM((singleg2o_save_dir / json_file.stem()).string() + "_GT.tum");
    data_gen.save_singleTUM((singleg2o_save_dir / json_file.stem()).string() + ".tum",
                            false);
  }
}
}  // namespace kollagen
