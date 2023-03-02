#include "kollagen/generate.h"

int main(int argc, char *argv[]) {
  bool multig2o = true;
  bool singleg2o = true;
  auto [json_file, save_dir] = kollagen::get_CLI_arguments(argc, argv);

  if (!kollagen::CLI_arguments_OK(json_file, save_dir, multig2o, singleg2o)) {
    return EXIT_FAILURE;
  }

  kollagen::generate_and_save(json_file, save_dir, multig2o, singleg2o);

  return EXIT_SUCCESS;
}
