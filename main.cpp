#include "grid.hpp"


int main(int argc, char* argv[]) {
  using namespace maze_search;

  maze m(3,2);

  std::cout << m << std::endl;

  vertex_iterator vi, vi_end;
  for(tie(vi, vi_end) = vertices(m); vi != vi_end; vi++) {
    vertex_descriptor u = *vi;
    std::cout << u << std::endl;

    out_edge_iterator ei, ei_end;
    for(tie(ei, ei_end) = out_edges(u, m); ei != ei_end; ei++) {
      edge_descriptor e = *ei;
      std::cout << "\t" << e
                << " weight " << get(boost::edge_weight, m, e)
                << std::endl;
    }
  }

  return 0;
}
