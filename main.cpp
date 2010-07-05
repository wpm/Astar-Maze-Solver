#include "grid.hpp"


int main(int argc, char* argv[]) {
  using namespace maze_search;

  maze m(3,2);
  vertex_descriptor start = vertex(0, m), goal = vertex(num_vertices(m), m);
  
  try {
    astar_search(m,
      start,
      euclidean_heuristic(goal),
      boost::visitor(astar_goal_visitor(goal)) );
  } catch(found_goal fg) {
    std::cout << "Found vertex " << goal << std::endl;
  }

  return 0;
}
