#include "grid.hpp"


int main(int argc, char* argv[]) {
  using namespace maze_search;

  maze m(3,3);
  position u(1,1);
  
  out_edge_iterator ei, ei_end;
  for(tie(ei, ei_end) = out_edges(u, m); ei != ei_end; ei++) {
    edge_descriptor  e = *ei;
    std::cout << e << std::endl;
  }
  
  return 0;
}
