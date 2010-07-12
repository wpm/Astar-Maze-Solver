#include <boost/graph/astar_search.hpp>
#include <boost/iterator/counting_iterator.hpp>
#include <ctime>
#include <iostream>
#include <map>
#include <set>


// Forward declaration
class outgoing_edge_iterator;
class maze_vertex_iterator;

// This is an ordered (x,y) pair.
class ordered_pair: public std::pair<int, int> {
friend std::ostream& operator<<(std::ostream& output, const ordered_pair& p) {
  output << "(" << p.first << ", " << p.second << ")";
  return output;
}

public:
  ordered_pair():std::pair<int, int>(0, 0) {};
  ordered_pair(int x, int y):std::pair<int, int>(x, y) {};

  ordered_pair& operator+=(const ordered_pair &rhs) {
    this->first  += rhs.first;
    this->second += rhs.second;
    return *this;
  }

  const ordered_pair operator+(const ordered_pair &other) const {
    ordered_pair result = *this;
    result += other;
    return result;
  }
};

// Distance traveled in the maze
typedef double distance;

// Tag values that specify the traversal type in graph::traversal_category.
struct maze_traversal_catetory:
  virtual public boost::vertex_list_graph_tag,
  virtual public boost::incidence_graph_tag
  {};

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
  // Graph associated types
  typedef ordered_pair vertex_descriptor;
  typedef boost::directed_tag directed_category;
  typedef boost::disallow_parallel_edge_tag edge_parallel_category;
  typedef maze_traversal_catetory traversal_category;

  // IncidenceGraph associated types
  typedef std::pair<vertex_descriptor, vertex_descriptor> edge_descriptor;
  typedef outgoing_edge_iterator out_edge_iterator;
  typedef std::size_t degree_size_type;

  // VertexListGraph associated types
  typedef maze_vertex_iterator vertex_iterator;
  typedef std::size_t vertices_size_type;

  // These types are not used but must be defined for graph_traits<...>.
  typedef void adjacency_iterator;
  typedef void in_edge_iterator;
  typedef void edge_iterator;
  typedef void edges_size_type;

  // A set of vertices
  typedef std::set<vertex_descriptor> vertex_set;

  friend std::ostream& operator<<(std::ostream&, const maze&);
  friend maze random_maze(vertices_size_type, vertices_size_type);

  maze():m_x(0),m_y(0) {};
  maze(vertices_size_type x, vertices_size_type y):m_x(x),m_y(y) {};

  vertices_size_type x() const {return m_x;}
  vertices_size_type y() const {return m_y;}

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

  // Return the next vertex in the specified direction.  Return u if u is at
  // the edge of the grid.
  vertex_descriptor next(vertex_descriptor u, std::size_t direction) const {
    vertex_descriptor v = u;
    if (direction == 0) {
      if (u.first != m_x)
        v += ordered_pair(1, 0);
    }
    else {
      if (u.second != m_x)
        v += ordered_pair(0, 1);
    }
    return v;
  }

private:
  // Size of the x dimension
  std::size_t m_x;
  // Size of the y dimension
  std::size_t m_y;
  // The set of barriers in the maze
  vertex_set m_barriers;
  // The set of vertices on a path through the maze
  vertex_set m_path;
  // The length of the solution path
  distance m_path_length;
};

// Use these parameterizations to refer to the maze graph types.
typedef boost::graph_traits<maze>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<maze>::edge_descriptor edge_descriptor;
typedef boost::graph_traits<maze>::out_edge_iterator out_edge_iterator;
typedef boost::graph_traits<maze>::degree_size_type degree_size_type;
typedef boost::graph_traits<maze>::vertices_size_type vertices_size_type;
typedef boost::graph_traits<maze>::vertex_iterator vertex_iterator;

// Tag values passed to an iterator constructor to specify whether it should
// be created at the start or the end of its range.
struct iterator_position {};
struct iterator_start:virtual public iterator_position {};
struct iterator_end:virtual public iterator_position {};

// Iterator over adjacent vertices in a grid
//
// This iterates into vertices above, below, to the left of and to the right
// of the current vertex, remaining on the grid and not going into barrier
// vertices.
//
// It is an adaptation of a counting iterator, which is used to point into
// the static list of offsets in the current_offset function.
class outgoing_edge_iterator:public boost::iterator_adaptor <
  outgoing_edge_iterator,
  boost::counting_iterator<std::size_t>,
  edge_descriptor,
  boost::use_default,
  edge_descriptor >
{
public:
  outgoing_edge_iterator():
    outgoing_edge_iterator::iterator_adaptor_(4),
    m_maze(NULL),m_u(ordered_pair()) {};
  explicit outgoing_edge_iterator(const maze& m,
                                  vertex_descriptor u,
                                  iterator_start):
    outgoing_edge_iterator::iterator_adaptor_(0),
    m_maze(&m),m_u(u) {
      find_next_valid_offset();
    };
  explicit outgoing_edge_iterator(const maze& m,
                                  vertex_descriptor u,
                                  iterator_end):
    outgoing_edge_iterator::iterator_adaptor_(4),
    m_maze(&m),m_u(u) {};

private:
  friend class boost::iterator_core_access;

  void increment() {
    this->base_reference()++;
    find_next_valid_offset();
  }

  edge_descriptor dereference() const {
    return edge_descriptor(m_u, m_u + current_offset());
  }

  // Cycle through the offsets until we either find one that points to an
  // accessible adjacent vertex or we have tried them all.
  void find_next_valid_offset() {
    while (invalid() && *this->base_reference() < 4)
      this->base_reference()++;
  }

  ordered_pair current_offset () const {
    static const ordered_pair offset[] = {
      ordered_pair(0, -1),  // Down
      ordered_pair(1, 0),   // Right
      ordered_pair(0, 1),   // Up
      ordered_pair(-1, 0)}; // Left
    return offset[*this->base_reference()];
  }

  // An offset is invalid if it lands on a barrier or off the grid.
  bool invalid() {
    ordered_pair v = m_u + current_offset();
    if (m_maze->has_barrier(v))
      return true;
    if (v.first < 0 || v.first == m_maze->x())
      return true;
    if (v.second < 0 || v.second == m_maze->y())
      return true;
    return false;
  }

  // A maze containing vertices to be iterated
  const maze *m_maze;
  // Vertex whose out edges are iterated
  vertex_descriptor m_u;
};

// IncidenceGraph
vertex_descriptor source(edge_descriptor e, const maze&) {
  // The first vertex in the edge pair is the source.
  return e.first;
}

vertex_descriptor target(edge_descriptor e, const maze& m) {
 // The second vertex in the edge pair is the target.
 return e.second;
}

std::pair<out_edge_iterator, out_edge_iterator>
out_edges(vertex_descriptor u, const maze& m) {
  return std::pair<out_edge_iterator, out_edge_iterator>(
    out_edge_iterator(m, u, iterator_start()),
    out_edge_iterator(m, u, iterator_end()) );
}

degree_size_type out_degree(vertex_descriptor u, const maze& m) {
  degree_size_type d = 4;

  if (u.first == 0 || u.first == m.x())
    d -= 1;
  if (u.second == 0 || u.second == m.y())
    d -= 1;
  return d;
}

// VertexListGraph
vertex_descriptor vertex(vertices_size_type i, const maze& m) {
  return vertex_descriptor(i % m.x(), i/m.x());
}

vertices_size_type num_vertices(const maze& m) {
  return m.x() * m.y();
}

// Iterator over all the vertices in the maze grid.
//
// This is an adaptation of a counting iterator that ranges over all the
// vertex indexes.
class maze_vertex_iterator:public boost::iterator_adaptor <
  maze_vertex_iterator,
  boost::counting_iterator<vertices_size_type>,
  vertex_descriptor,
  boost::use_default,
  vertex_descriptor >
{
public:
  maze_vertex_iterator():
    maze_vertex_iterator::iterator_adaptor_(0), m_maze(NULL) {};
  explicit maze_vertex_iterator(const maze& m,
                                iterator_start):
    maze_vertex_iterator::iterator_adaptor_(0),m_maze(&m) {};
  explicit maze_vertex_iterator(const maze& m,
                                iterator_end):
    maze_vertex_iterator::iterator_adaptor_(num_vertices(m)),m_maze(&m) {};

private:
  friend class boost::iterator_core_access;

  vertex_descriptor dereference() const {
    return vertex(*this->base_reference(), *m_maze);
  }

  const maze *m_maze;
};

std::pair<vertex_iterator, vertex_iterator> vertices(const maze& m) {
  return std::pair<vertex_iterator, vertex_iterator>(
    vertex_iterator(m, iterator_start()),
    vertex_iterator(m, iterator_end()) );
}

// Vertex-to-vertex mapping used by the predecessor map
typedef std::map<vertex_descriptor, vertex_descriptor> pred_map;
// Vetex-to-distance mapping used by the distance map
typedef std::map<vertex_descriptor, distance> dist_map;

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

// Readable Property Map for vertex indices.
class vertex_index_pmap {
public:
  typedef vertices_size_type value_type;
  typedef value_type reference;
  typedef vertex_descriptor key_type;
  typedef boost::readable_property_map_tag category;

  vertex_index_pmap():m_maze(NULL) {};
  vertex_index_pmap(const maze& m):m_maze(&m) {};

  value_type operator[](key_type u) const {
    return u.first + m_maze->x()*u.second;
  }

private:
  const maze* m_maze;
};

typedef boost::property_traits<vertex_index_pmap>::value_type
        vertex_index_pmap_value;
typedef boost::property_traits<vertex_index_pmap>::key_type
        vertex_index_pmap_key;

vertex_index_pmap_value get(vertex_index_pmap pmap, vertex_index_pmap_key u) {
  return pmap[u];
}

// Euclidean heuristic for a grid
//
// This calculates the Euclidean distance between a vertex and a goal
// vertex.
class euclidean_heuristic:public boost::astar_heuristic<maze, distance> {
public:
  euclidean_heuristic(vertex_descriptor goal):m_goal(goal) {};

  distance operator()(vertex_descriptor v) {
    return sqrt(pow(m_goal.first - v.first, 2) +
                pow(m_goal.second - v.second, 2));
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
  void examine_vertex(vertex_descriptor u, const maze&) {
    if (u == m_goal)
      throw found_goal();
  }

private:
  vertex_descriptor m_goal;
};

// Solve the maze using A-star search.  Return true if a solution was found.
bool maze::solve() {
  edge_weight_pmap weight;
  pred_map predecessor;
  boost::associative_property_map<pred_map> pred_pmap(predecessor);
  dist_map distance;
  boost::associative_property_map<dist_map> dist_pmap(distance);

  // Try to find a path from the lower-left-hand corner (0,0) to the
  // upper-right-hand corner (x-1, y-1).
  ordered_pair source = vertex(0, *this);
  ordered_pair goal = vertex(num_vertices(*this)-1, *this);
  vertex_index_pmap index(*this);
  euclidean_heuristic heuristic(goal);
  astar_goal_visitor visitor(goal);

  try {
    astar_search(*this,
                 source,
                 heuristic,
                 boost::vertex_index_map(index).
                 weight_map(weight).
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

// Print edges as (x1, y1) -> (x2, y2).
std::ostream& operator<<(std::ostream& output, const edge_descriptor& e) {
  output << e.first << " -> " << e.second;
  return output;
}

#define BARRIER "#"
// Print the maze as an ASCII map.
std::ostream& operator<<(std::ostream& output, const maze& m) {
  // Header
  for (vertices_size_type i = 0; i < m.m_x + 2; i++)
    output << BARRIER;
  output << std::endl;
  // Body
  // Enumerate rows in reverse order and columns in regular order so that
  // (0,0) appears in the lower left-hand corner.  This requires that y be int
  // and not the unsigned vertices_size_type because the loop exit condition
  // is y==-1.
  for (int y = m.m_y-1; y >= 0; y--) {
    for (vertices_size_type x = 0; x < m.m_x; x++) {
      // Put a barrier on the left-hand side.
      if (x == 0)
        output << BARRIER;
      // Put the character representing this point in the maze grid.
      vertex_descriptor u = vertex_descriptor(x, y);
      if (m.m_path.find(u) != m.m_path.end())
        output << ".";
      else if (m.has_barrier(u))
        output << BARRIER;
      else
        output << " ";
      // Put a barrier on the right-hand side.
      if (x == m.m_x-1)
        output << BARRIER;
    }
    // Put a newline after every row except the last one.
    output << std::endl;
  }
  // Footer
  for (vertices_size_type i = 0; i < m.m_x + 2; i++)
    output << BARRIER;
  if (m.solved())
    output << std::endl << "Solution length " << m.m_path_length;
  return output;
}

// Generate a maze with a random assorted of barriers.
maze random_maze(vertices_size_type x, vertices_size_type y) {
  maze m(x, y);
  vertices_size_type n = num_vertices(m);
  vertex_descriptor source = vertex(0, m);
  vertex_descriptor goal = vertex(n - 1, m);
  // About one third of the maze should be barriers.
  int barriers = n/3;
  while (barriers > 0) {
    // Choose horizontal or vertical direction.
    std::size_t direction = std::rand() % 2;
    // Walls should be about one quarter of the maze size in this direction.
    vertices_size_type length = (direction == 0 ? m.m_x:m.m_y)/4;
    if (length == 0)
      length = 1;
    // Create the wall while decrementing the total barrier count.
    vertex_descriptor u = vertex(std::rand() % n, m);
    while (length) {
      // Start and goal spaces should never be barriers.
      if (u != source && u != goal) {
        m.m_barriers.insert(u);
        barriers--;
        length--;
      }
      vertex_descriptor v = m.next(u, direction);
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
