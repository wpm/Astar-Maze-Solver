#include <boost/graph/astar_search.hpp>
#include <boost/iterator/counting_iterator.hpp>
#include <iostream>
// #include <math.h>

namespace maze_search {
  // Forward declaration
  class outgoing_edge_iterator;

  // A position in the maze grid.
  //
  // This is an ordered (x,y) pair.
  class position: public std::pair<int, int> {
  friend std::ostream& operator<<(std::ostream& output, const position& p) {
    output << "(" << p.first << ", " << p.second << ")";
    return output;
  }

  public:
    position():std::pair<int, int>(0, 0) {};
    position(int x, int y):std::pair<int, int>(x, y) {};

    position& operator+=(const position &rhs) {
      this->first  += rhs.first;
      this->second += rhs.second;
      return *this;
    }

    const position operator+(const position &other) const {
      position result = *this;
      result += other;
      return result;
    }
  };


  // A searchable maze
  class maze {
    friend std::ostream& operator<<(std::ostream& output, const maze& m) {
      for (int y = m.m_y-1; y >= 0; y--) {
        for (vertices_size_type x = 0; x < m.m_x; x++) {
          if (x != 0)
            output << " ";
          output << ".";
          if (x != m.m_x-1)
            output << " ";
        }
        if (y != 0)
          output << std::endl;
      }
      return output;
    }
  public:
    // Graph associated types
    typedef position vertex_descriptor;
    typedef boost::directed_tag directed_category;
    typedef boost::disallow_parallel_edge_tag edge_parallel_category;
    typedef boost::vertex_list_graph_tag traversal_category;

    // IncidenceGraph associated types
    typedef std::pair<vertex_descriptor, vertex_descriptor> edge_descriptor;
    typedef outgoing_edge_iterator out_edge_iterator;
    typedef size_t degree_size_type;

    typedef size_t vertices_size_type;


    // Needed for graph_traits
    typedef void adjacency_iterator;
    typedef void in_edge_iterator;
    typedef void vertex_iterator;
    typedef void edge_iterator;
    typedef void edges_size_type;

    maze():m_x(0),m_y(0) {};
    maze(vertices_size_type x, vertices_size_type y):m_x(x),m_y(y) {};

    vertices_size_type x() const {return m_x;}
    vertices_size_type y() const {return m_y;}

  private:
    size_t m_x;
    size_t m_y;
  };

  typedef boost::graph_traits<maze>::vertex_descriptor vertex_descriptor;
  typedef boost::graph_traits<maze>::edge_descriptor edge_descriptor;
  typedef boost::graph_traits<maze>::out_edge_iterator out_edge_iterator;
  typedef boost::graph_traits<maze>::degree_size_type degree_size_type;

  // Tag values passed to an iterator constructor to specify whether it should
  // be created at the start or the end of its range.
  struct iterator_position {};
  struct iterator_start:virtual public iterator_position {};
  struct iterator_end:virtual public iterator_position {};


  // IncidenceGraph
  
  // Iterator over adjacent vertices in a grid
  class outgoing_edge_iterator:public boost::iterator_adaptor <
    outgoing_edge_iterator,
    boost::counting_iterator<size_t>,
    edge_descriptor,
    boost::use_default,
    edge_descriptor >
  {
  public:
    outgoing_edge_iterator():
      outgoing_edge_iterator::iterator_adaptor_(4),
      m_maze(NULL),m_u(position()) {};
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
      m_maze(&m),m_u(u) {
        find_next_valid_offset();
      };

  private:
    friend class boost::iterator_core_access;

    void increment() {
      this->base_reference()++;
      find_next_valid_offset();
    }

    edge_descriptor dereference() const {
      return edge_descriptor(m_u, m_u + current_offset());
    }

    void find_next_valid_offset() {
      while (off_grid() && *this->base_reference() < 4)
        this->base_reference()++;
    }

    position current_offset () const {
      static const position offset[] = {
        position(0, -1),
        position(1, 0),
        position(0, 1),
        position(-1, 0)};
      return offset[*this->base_reference()];
    }

    bool off_grid() {
      position v = m_u + current_offset();
      if (v.first < 0 || v.first == m_maze->x())
        return true;
      if (v.second < 0 || v.second == m_maze->y())
        return true;
      return false;
    }

    const maze *m_maze;
    vertex_descriptor m_u; // Vertex whose out edges are iterated
  };

  
  vertex_descriptor source(edge_descriptor, const maze&);

  inline vertex_descriptor
  source(edge_descriptor e, const maze&) {
    // The first vertex in the edge pair is the source.
    return e.first;
  }

  vertex_descriptor target(edge_descriptor, maze&);

  inline vertex_descriptor
  target(edge_descriptor e, const maze&) {
   // The second vertex in the edge pair is the target.
   return e.second;
  }

  std::pair<out_edge_iterator, out_edge_iterator>
  out_edges(vertex_descriptor, const maze&);

  inline std::pair<out_edge_iterator, out_edge_iterator>
  out_edges(vertex_descriptor u, const maze& m) {
    return std::pair<out_edge_iterator, out_edge_iterator>(
      out_edge_iterator(m, u, iterator_start()),
      out_edge_iterator(m, u, iterator_end()) );
  }

  degree_size_type out_degree(vertex_descriptor, const maze&);

  inline degree_size_type out_degree(vertex_descriptor u, const maze& m) {
    degree_size_type d = 4;

    if (u.first == 0 || u.first == m.x())
      d -= 1;
    if (u.second == 0 || u.second == m.y())
      d -= 1;
    return d;
  }


  inline std::ostream&
  operator<<(std::ostream& output, const edge_descriptor& e) {
    output << e.first << " -> " << e.second;
    return output;
  }

}
