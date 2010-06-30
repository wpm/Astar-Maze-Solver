#include <iostream>
#include "grid.hpp"


int main(int argc, char* argv[]) {
  using namespace boost;

  function_requires< ReadablePropertyMapConcept<const_edge_weight_map,
                                                edge_descriptor> >();
  function_requires< ReadablePropertyGraphConcept<weighted_grid,
                                                  edge_descriptor,
                                                  edge_weight_t> >();
  function_requires< VertexListGraphConcept<weighted_grid> >();
  function_requires< AStarHeuristicConcept<euclidean_heuristic,
                                           weighted_grid> >();

  array<std::size_t, 2> dimensions = { {3, 4} };
  weighted_grid g(dimensions);

  // Edge weights are implemented.
  const_edge_weight_map edge_map = get(edge_weight, g);
  edge_descriptor e = edge_at(0, g);
  edge_weight_map_reference w = edge_map[e];
  std::cout << e << " weight " << w << std::endl;

  vertex_descriptor source = vertex(0, g), goal = vertex(3, g);
  std::cout << "Source " << source << "  Goal " << goal << std::endl;

  // Vertex indexes are implemented.
  std::cout << source << " index " << get(vertex_index, g)[source]
            << std::endl;

  // astar_search(g,
  //              source,
  //              euclidean_heuristic(goal),
  //              visitor(astar_goal_visitor(goal)) );
  return 0;
}
