# Breadth First Search, Uniform Cost Search and A* Search

A program that will take an input file that describes the terrain map, landing site, target sites, and characteristics of the robot. For each target site, we find the optimal(shortest) safe path from the landing site to that target. A path is composed of a sequence of elementary moves. Each elementary move consists of moving the rover to one of its 8 neighbors.

To find the solution we will use the following algorithms:
- Breadth-first search
- Uniform-cost search
- A* search
If an optimal path cannot be found, your algorithm should return "FAIL"

PS. This was homework #1 of CSCI 561 - Fall 2019 - Foundations of Artificial Intelligence - under Professor Laurent Itti.

## input.txt Format

- **First line:** Instruction of which algorithm to use, as a string: BFS, UCS or A*
- **Second line:** Two strictly positive 32-bit integers separated by one space character, for "W H" the number of columns (width) and rows (height), in cells, of the map.
- **Third line:** Two positive 32-bit integers separated by one space character, for "X Y" the coordinates (in cells) of the landing site. 0 ≤ X ≤ W-1 and 0 ≤ Y ≤ H-1 (that is, we use 0-based indexing into the map; X increases when moving East and Y increases when moving South; (0,0) is the North West corner of the map).
- **Fourth line:** Positive 32-bit integer number for the maximum difference in elevation between two adjacent cells which the rover can drive over. The difference in Z between two adjacent cells must be smaller than or equal(≤ ) to this value for the rover to be able to travel from one cell to the other.
- **Fifth line:** Strictly positive 32-bit integer N, the number of target sites.
- **Next N lines:** Two positive 32-bit integers separated by one space character, for "X Y" the coordinates (in cells) of each target site. 0 ≤ X ≤ W-1 and 0 ≤ Y ≤ H-1 (that is, we again use 0-based indexing into the map).
- **Next H lines:** W 32-bit integer numbers separated by any numbers of spaces for the elevation (Z) values of each of the W cells in each row of the map.

Here is a sample input.txt file:
```
A*
8 6
4 4
7
2
1 1
6 3
0 0 0 0 0 0 0 0
0 60 64 57 45 66 68 0
0 63 64 57 45 67 68 0
0 58 64 57 45 68 67 0
0 60 61 67 65 66 69 0
0 0 0 0 0 0 0 0
```

## output.txt Format

This file is auto-generated on runtime. 
- **N lines:** Here we report the paths in the same order as the targets were given in the input.txt file. We write out one line per target. Each line should contain a sequence of X,Y pairs of coordinates of cells visited by the rover to travel from the landing site to the corresponding target site for that line. If no solution was found (target site unreachable by rover from given landing site), we write a single word FAIL in the corresponding line.

Here is a sample output.txt file:
```
4,4 3,4 2,3 2,2 1,1
4,4 5,4 6,3
```