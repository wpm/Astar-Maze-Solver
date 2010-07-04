#include <iostream>
#include "grid.hpp"


int main(int argc, char* argv[]) {
  using namespace boost;

  function_requires< ReadablePropertyMapConcept<const_edge_weight_map,
                                                edge_descriptor> >();
  function_requires< ReadablePropertyGraphConcept<maze,
                                                  edge_descriptor,
                                                  edge_weight_t> >();
  function_requires< VertexListGraphConcept<maze> >();
  function_requires< AStarHeuristicConcept<euclidean_heuristic,
                                           maze> >();

  dimension_array dimensions = { {3, 4} };
  maze g(dimensions);
  vertex_descriptor source = vertex(0, g), goal = vertex(3, g);
  // astar_search(g,
  //              source,
  //              euclidean_heuristic(goal),
  //              visitor(astar_goal_visitor(goal)) );

  // Edge weights are implemented.
  const_edge_weight_map edge_map = get(edge_weight, g);
  edge_descriptor e = edge_at(0, g);
  edge_weight_map_reference w = edge_map[e];
  std::cout << e << " weight " << w << std::endl;

  // Vertex indexes are implemented.
  std::cout << source << " index " << get(vertex_index, g)[source]
            << std::endl;

  return 0;
}
