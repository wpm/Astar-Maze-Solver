#include <iostream>
#include "grid.hpp"


int main(int argc, char* argv[]) {
  using namespace grid;
  
  dimension_array dimensions = { {3, 4} };
  graph g(dimensions);

  edge_descriptor e = edge_at(0, g);
  edge_weight_map_reference w = get(boost::edge_weight, g, e);
  std::cout << "Edge weight " << w << std::endl;

  return 0;
}
