#include "json.hpp"
#include <fstream>

using json = nlohmann::json;

inline json open_json(const std::string& filename){
  std::ifstream f(filename);
  json data = json::parse(f);
  return data;
}

inline void parse_json(const json& data){
  (void) data;
}
