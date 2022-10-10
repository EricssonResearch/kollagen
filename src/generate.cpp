#include <iostream>
#include <tuple>
#include <filesystem>

#include "kollagen.h"

void print_help() {
  std::cout << "NAME\n"
               "   generate - generate data from json file\n\n"
               "SYNOPSIS\n"
               "       generate SOURCE [DEST]\n\n"
               "DESCRIPTION\n"
               "       Generate data from json file SOURCE. Save in folder "
               "DEST, defaulting to './output'.\n";
}

std::tuple<std::filesystem::path, std::filesystem::path> get_CLI_arguments(int argc, char *argv[]){
  std::filesystem::path json_file{};
  std::filesystem::path save_dir{};

  if (argc == 1) {
    print_help();
  } 

  if (argc > 1) {
    json_file = argv[1];
    save_dir = "./output";
    if (json_file.extension() != "json") {
      json_file.replace_extension("json");
    }
    if (json_file == "-h" || json_file == "--help") {
      print_help();
    }
  } 

  if (argc > 2) {
    save_dir = argv[2];
  }

  return {json_file, save_dir};
}

int main(int argc, char *argv[]) {
  auto [json_file, save_dir] = get_CLI_arguments(argc, argv);

  auto params = get_params_from_json(json_file.string());
  auto data_gen = DataGenerator(params);
  data_gen.generate();

  data_gen.save_multig2o(save_dir/json_file.stem());
  data_gen.save_singleg2o((save_dir/json_file.stem()).string() + ".g2o", false);
  data_gen.save_singleTUM((save_dir/json_file.stem()).string() + "_GT.tum");
  data_gen.save_singleTUM((save_dir/json_file.stem()).string() + ".tum", false);

  return EXIT_SUCCESS;
}
