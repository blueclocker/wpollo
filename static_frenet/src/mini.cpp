#include "matplotlibcpp.h"
#include <vector>

namespace plt = matplotlibcpp;

int main() {
  std::vector<double> y = {1, 3, 2, 4};
  plt::plot(y);
  plt::show();
}
