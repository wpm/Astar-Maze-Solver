#include "grid.hpp"


int main(int argc, char* argv[]) {
  using namespace maze_search;

  maze m(3,3);
  position u(1,1);

  const_vertex_index_map vertex_index = get(boost::vertex_index, m);

  out_edge_iterator ei, ei_end;
  for(tie(ei, ei_end) = out_edges(u, m); ei != ei_end; ei++) {
    edge_descriptor e = *ei;
    vertex_descriptor v = target(e, m);
    vertices_size_type index = vertex_index[v];
    std::cout << e
              << " weight " << get(boost::edge_weight, m, e)
              << " index "  << index
              << std::endl;
  }

  std::cout << m << std::endl;

  return 0;
}
