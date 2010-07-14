A-Star Maze Solver
==================

This program uses the A-star search algorithm in the Boost Graph Library to solve a maze.  It is an example of how to apply Boost Graph Library algorithms to implicit graphs.

The solver consists of a single program called `astar-maze`.  When run this program generates a random maze and then tries to find the shortest path from the lower left-hand corner to the upper right-hand corner.  Mazes are represented by two-dimensional grids where a cell in the grid may contain a barrier.  You may move up, down, right, or left to any adjacent cell that does not contain a barrier.

Once a maze solution has been attempted, the maze is printed.  If a solution was found it will be shown in the maze printout and its length will be returned.  Note that not all mazes have solutions.

The default maze size is 20x10, though different dimensions may be specified on the command line.


Installation
============

You must have the Boost Graph Library version 1.44 or higher installed.  Set BOOST\_PATH in the Makefile to point to the root of your Boost include tree.  The `astar-maze` program builds in the distribution directory and does not install anywhere else on the system.


Implementation
==============

The maze is represented by the `maze` class.  This class has a [grid graph](http://www.boost.org/doc/libs/1_41_0/libs/graph/doc/grid_graph.html "Grid Graph") member which defines the underlying grid.  All edges are assigned a weight of one.  Barriers in the maze are removed using a [graph filter](http://www.boost.org/doc/libs/1_43_0/libs/graph/doc/filtered_graph.html "Filtered Graph").  The maze is searched using the [A* search](http://www.boost.org/doc/libs/1_38_0/libs/graph/doc/astar_search.html "A* Search") algorithm.


History
=======

* **Version 1.0.0**: Initial release


Copyright
=========

Copyright [W.P. McNeill](mailto:billmcn@gmail.com) 2010.

Distributed under the Boost Software License, Version 1.0.

See accompanying file LICENSE\_1\_0.txt or copy at [http://www.boost.org/LICENSE\_1\_0.txt](http://www.boost.org/LICENSE_1_0.txt "Boost License 1.0").
