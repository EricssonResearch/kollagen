#ifndef UTILS_HPP
#define UTILS_HPP

#include <fstream>
#include <algorithm>
#include <cmath>
#include <vector>
#include <string>
#include <memory>
#include <stdexcept>

namespace kollagen
{

inline double normalize_angle(double angle) {
  return angle - (std::floor((angle + M_PI) / (2 * M_PI))) * 2 * M_PI;
}

inline int round_to_int(double expression) {
  return static_cast<int>(round(expression));
}

// Save a vector of type T to binary file
template<class T>
inline int save_binary(std::string filename, std::vector<T> data)
{
  std::ofstream file(filename, std::ios_base::out | std::ios_base::binary);
  if (!file) { return EXIT_FAILURE; }
  file.write(reinterpret_cast <char*>(&data.front()), data.size() * sizeof(data.front()));
  return EXIT_SUCCESS;
}

// Load a binary file made from vector of type T
template<class T>
inline int load_binary(std::string filename, std::vector<T>& data)
{
  std::ifstream load_file(filename, std::ios_base::in | std::ios_base::binary | std::ios_base::ate);
  if (!load_file) { return EXIT_FAILURE; }
  auto filesize = load_file.tellg();
  auto typesize = sizeof(data.front());
  auto n_elements = filesize / typesize;
  data.resize(n_elements);
  std::fill_n(data.begin(), filesize / typesize, 0.0);
  std::ifstream file(filename, std::ios_base::in | std::ios_base::binary);
  file.read(reinterpret_cast <char*>(&data.front()), filesize);
  return EXIT_SUCCESS;
}

template<class T>
inline std::vector<T> load_binary(std::string filename)
{
  std::vector<T> data;
  load_binary(filename, data);
  return data;
}

template<typename ... Args>
std::string string_format(const std::string& format, Args ... args )
{
    int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
    auto size = static_cast<size_t>( size_s );
    auto buf = std::make_unique<char[]>( size );
    std::snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

}  // namespace kollagen
#endif /* !UTILS_HPP */
