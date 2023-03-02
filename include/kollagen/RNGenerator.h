#include <random>

#ifndef RNGENERATOR_H
#define RNGENERATOR_H

namespace kollagen
{

class RNGenerator {
public:
  RNGenerator() = default;
  explicit RNGenerator(int seed, bool allow_retrocession = true)
      : gen(seed), generator(0.0, 1.0),
        int_generator(-1, 1 + static_cast<int>(allow_retrocession)),
        uniform_generator(0.0, 1.0), seed_(seed){};
  [[nodiscard]] double draw_normal() { return generator(gen); };
  [[nodiscard]] double draw_uniform() { return uniform_generator(gen); };
  [[nodiscard]] int draw_int() { return int_generator(gen); };
  void seed(int new_seed) { gen.seed(new_seed); };
  [[nodiscard]] int seed() const { return seed_; };

private:
  std::mt19937 gen;
  std::normal_distribution<double> generator;
  std::uniform_int_distribution<int> int_generator;
  std::uniform_real_distribution<double> uniform_generator;
  int seed_;
};

}  // namespace kollagen
#endif /* !RNGENERATOR_H */
