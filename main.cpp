#include <iostream>
#include "grid.hpp"


int main(int argc, char* argv[]) {
  using namespace boost;
  
  dimension_array dimensions = { {3, 4} };
  barrier_grid g(dimensions);

  // Edge weights are implemented.
  edge_descriptor e = edge_at(0, g);
  edge_weight_map_reference w = get(boost::edge_weight, g, e);
  std::cout << e << " weight " << w << std::endl;
  
  vertex_descriptor source = vertex(0, g), goal = vertex(3, g);
  std::cout << "Source " << source << "  Goal " << goal << std::endl;
  
  // Vertex indexes are implemented.
  barrier_grid::vertex_index i = get(vertex_index, g)[source];
  std::cout << source << " index " << i << std::endl;

  // astar_search(g,
  //              source,
  //              euclidean_heuristic(goal),
  //              visitor(astar_goal_visitor(goal)) );
  return 0;
}
