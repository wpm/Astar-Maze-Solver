
//          Copyright W.P. McNeill 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <boost/graph/astar_search.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/grid_graph.hpp>
#include <ctime>
#include <iostream>
#include <map>
#include <set>


// Distance traveled in the maze
typedef double distance;

// A searchable maze
//
// The maze is grid of locations which can either be empty or contain a
// barrier.  You can move to an adjacent location in the grid by going up,
// down, left and right.  Moving onto a barrier is not allowed.  The maze can
// be solved by finding a path from the lower-left-hand corner to the
// upper-right-hand corner.  If no open path exists between these two
// locations, the maze is unsolvable.
//
// The maze is implemented as an implicit graph where locations are vertices
// and edges connect adjacent vertices that are not blocked by barriers.
// A-star search is used to find a path through the maze. Each edge has a
// weight of one, so the total path length is equal to the number of edges
// traversed.
class maze {
public:
  typedef boost::grid_graph<2> grid;
  typedef boost::graph_traits<grid>::vertex_descriptor vertex_descriptor;
  typedef boost::graph_traits<grid>::vertices_size_type vertices_size_type;

  typedef std::set<vertex_descriptor> vertex_set;
  typedef boost::vertex_subset_compliment_filter<grid, vertex_set>::type
          filtered_grid;

  friend std::ostream& operator<<(std::ostream&, const maze&);
  friend maze random_maze(std::size_t, std::size_t);

  maze():m_grid(create_grid(0, 0)),m_barrier_grid(create_barrier_grid()) {};
  maze(std::size_t x, std::size_t y):m_grid(create_grid(x, y)),
       m_barrier_grid(create_barrier_grid()) {};

  vertices_size_type length(std::size_t d) const {return m_grid.length(d);}

  // If adding or removing a barrier changes the maze, any previously found
  // solution is discarded.
  void add_barrier(vertex_descriptor u) {
    if (!has_barrier(u))
      m_path.clear();
    m_barriers.insert(u);
  };
  void remove_barrier(vertex_descriptor u) {
    if (has_barrier(u))
      m_path.clear();
    m_barriers.erase(u);
  }
  bool has_barrier(vertex_descriptor u) const {
    return m_barriers.find(u) != m_barriers.end();
  }

  bool solve();
  bool solved() const {return !m_path.empty();}

private:
  // Create the underlying rank-2 grid with the specified dimensions.
  grid create_grid(std::size_t x, std::size_t y) {
    boost::array<std::size_t, 2> lengths = { {x, y} };
    return grid(lengths);
  }

  // Filter the vertices in the barrier set out of the underlying grid.
  filtered_grid create_barrier_grid() {
    return boost::make_vertex_subset_compliment_filter(m_grid, m_barriers);
  }
  
  // The grid on which this maze is drawn
  grid m_grid;
  // The maze grid with barrier vertices filtered out
  filtered_grid m_barrier_grid;
  // The set of barriers in the maze
  vertex_set m_barriers;
  // The set of vertices on a path through the maze
  vertex_set m_path;
  // The length of the solution path
  distance m_path_length;
};

typedef boost::graph_traits<maze::grid>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<maze::grid>::edge_descriptor edge_descriptor;
typedef boost::graph_traits<maze::grid>::vertices_size_type vertices_size_type;


// Readable Property Map for edge weights.
struct edge_weight_pmap {
  typedef distance value_type;
  typedef value_type reference;
  typedef edge_descriptor key_type;
  typedef boost::readable_property_map_tag category;
};

typedef boost::property_traits<edge_weight_pmap>::value_type
        edge_weight_pmap_value;
typedef boost::property_traits<edge_weight_pmap>::key_type
        edge_weight_pmap_key;

// All edges are one unit long.
edge_weight_pmap_value get(edge_weight_pmap, edge_weight_pmap_key) {
  return 1;
}

// namespace boost {
//   template<>
//   struct property_map<maze::filtered_grid, edge_weight_t> {
//     typedef edge_weight_pmap type;
//     typedef type const_type;
//   };
// }

edge_weight_pmap get(boost::edge_weight_t, const maze::grid&) {
  return edge_weight_pmap();
}


// Readable Property Map for vertex indices.
class vertex_index_pmap {
public:
  typedef vertices_size_type value_type;
  typedef value_type reference;
  typedef vertex_descriptor key_type;
  typedef boost::readable_property_map_tag category;

  vertex_index_pmap():m_grid(NULL) {};
  vertex_index_pmap(const maze::grid& g):m_grid(&g) {};

  value_type operator[](key_type u) const {
    // This is backwards from the documentation.
    return get(boost::vertex_index, *m_grid, u);
  }

private:
  const maze::grid* m_grid;
};

typedef boost::property_traits<vertex_index_pmap>::value_type
        vertex_index_pmap_value;
typedef boost::property_traits<vertex_index_pmap>::key_type
        vertex_index_pmap_key;

vertex_index_pmap_value get(vertex_index_pmap pmap, vertex_index_pmap_key u) {
  return pmap[u];
}

vertex_index_pmap get(boost::vertex_index_t, const maze::grid& g) {
  return vertex_index_pmap(g);
}


// Euclidean heuristic for a grid
//
// This calculates the Euclidean distance between a vertex and a goal
// vertex.
class euclidean_heuristic:
      public boost::astar_heuristic<maze::filtered_grid, distance>
{
public:
  euclidean_heuristic(vertex_descriptor goal):m_goal(goal) {};

  distance operator()(vertex_descriptor v) {
    return sqrt(pow(m_goal[0] - v[0], 2) +
                pow(m_goal[1] - v[1], 2));
  }

private:
  vertex_descriptor m_goal;
};

// Exception thrown when the goal vertex is found
struct found_goal {};

// Visitor that terminates when we find the goal vertex
struct astar_goal_visitor:public boost::default_astar_visitor {
  astar_goal_visitor(vertex_descriptor goal):m_goal(goal) {};

  // Need const otherwise we get no matching function error at:
  // /opt/local/include/boost/graph/astar_search.hpp:141
  void examine_vertex(vertex_descriptor u, const maze::filtered_grid&) {
    if (u == m_goal)
      throw found_goal();
  }

private:
  vertex_descriptor m_goal;
};


// Solve the maze using A-star search.  Return true if a solution was found.
bool maze::solve() {
  edge_weight_pmap weight = get(boost::edge_weight, m_grid);
  vertex_index_pmap index = get(boost::vertex_index, m_grid);
  // The predecessor map is a vertex-to-vertex mapping.
  typedef std::map<vertex_descriptor, vertex_descriptor> pred_map;
  pred_map predecessor;
  boost::associative_property_map<pred_map> pred_pmap(predecessor);
  // The distance map is a vertex-to-distance mapping.
  typedef std::map<vertex_descriptor, distance> dist_map;
  dist_map distance;
  boost::associative_property_map<dist_map> dist_pmap(distance);

  // Try to find a path from the lower-left-hand corner (0,0) to the
  // upper-right-hand corner (x-1, y-1).
  vertex_descriptor source = vertex(0, m_grid);
  vertex_descriptor goal = vertex(num_vertices(m_grid)-1, m_grid);
  euclidean_heuristic heuristic(goal);
  astar_goal_visitor visitor(goal);

  try {
    astar_search(m_barrier_grid,
                 source,
                 heuristic,
                 boost::weight_map(weight).
                 vertex_index_map(index).
                 predecessor_map(pred_pmap).
                 distance_map(dist_pmap).
                 visitor(visitor) );
  } catch(found_goal fg) {
    // Walk backwards from the goal through the predecessor chain adding
    // vertices to the solution path.
    for (vertex_descriptor u = goal; u != source; u = predecessor[u])
      m_path.insert(u);
    m_path.insert(source);
    m_path_length = distance[goal];
    return true;
  }

  return false;
}


#define BARRIER "#"
// Print the maze as an ASCII map.
std::ostream& operator<<(std::ostream& output, const maze& m) {
  // Header
  for (vertices_size_type i = 0; i < m.length(0) + 2; i++)
    output << BARRIER;
  output << std::endl;
  // Body
  // Enumerate rows in reverse order and columns in regular order so that
  // (0,0) appears in the lower left-hand corner.  This requires that y be int
  // and not the unsigned vertices_size_type because the loop exit condition
  // is y==-1.
  for (int y = m.length(1)-1; y >= 0; y--) {
    for (vertices_size_type x = 0; x < m.length(0); x++) {
      // Put a barrier on the left-hand side.
      if (x == 0)
        output << BARRIER;
      // Put the character representing this point in the maze grid.
      vertex_descriptor u = {x, y};
      if (m.m_path.find(u) != m.m_path.end())
        output << ".";
      else if (m.has_barrier(u))
        output << BARRIER;
      else
        output << " ";
      // Put a barrier on the right-hand side.
      if (x == m.length(0)-1)
        output << BARRIER;
    }
    // Put a newline after every row except the last one.
    output << std::endl;
  }
  // Footer
  for (vertices_size_type i = 0; i < m.length(0) + 2; i++)
    output << BARRIER;
  if (m.solved())
    output << std::endl << "Solution length " << m.m_path_length;
  return output;
}

// Generate a maze with a random assignment of barriers.
maze random_maze(vertices_size_type x, vertices_size_type y) {
  maze m(x, y);
  vertices_size_type n = num_vertices(m.m_grid);
  vertex_descriptor source = vertex(0, m.m_grid);
  vertex_descriptor goal = vertex(n - 1, m.m_grid);
  // About one third of the maze should be barriers.
  int barriers = n/3;
  while (barriers > 0) {
    // Choose horizontal or vertical direction.
    std::size_t direction = std::rand() % 2;
    // Walls should be about one quarter of the maze size in this direction.
    vertices_size_type wall = m.length(direction)/4;
    if (wall == 0)
      wall = 1;
    // Create the wall while decrementing the total barrier count.
    vertex_descriptor u = vertex(std::rand() % n, m.m_grid);
    while (wall) {
      // Start and goal spaces should never be barriers.
      if (u != source && u != goal) {
        m.m_barriers.insert(u);
        barriers--;
        wall--;
      }
      vertex_descriptor v = m.m_grid.next(u, direction);
      // Stop creating this wall if we reached the maze's edge.
      if (u == v)
        break;
      u = v;
    }
  }
  return m;
}


int main (int argc, char const *argv[]) {
  // The default maze size is 20x10.  A different size may be specified on
  // the command line.
  vertices_size_type x = 20;
  vertices_size_type y = 10;

  if (argc == 3) {
    x = atoi(argv[1]);
    y = atoi(argv[2]);
  }

  std::srand(std::time(0));
  maze m = random_maze(x, y);

  if (m.solve())
    std::cout << "Solved the maze." << std::endl;
  else
    std::cout << "The maze is not solvable." << std::endl;
  std::cout << m << std::endl;
  return 0;
}
